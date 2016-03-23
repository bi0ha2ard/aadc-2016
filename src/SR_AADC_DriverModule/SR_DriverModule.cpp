
#include "SR_DriverModule.h"

using namespace adtf;
#undef LOG_INFO
#define LOG_INFO(str) logger.GLogTrap((str));

#define PROP_CONTINUE "Continue after section end"

ADTF_FILTER_PLUGIN("SR AADC Driver Module", OID_ST_DRIVER_MODULE, SR_DriverModule);


SR_DriverModule::SR_DriverModule(const tChar* __info) : cFilter(__info),m_hTimer(NULL),
	logger(OID_ST_DRIVER_MODULE)
{
	m_bIDsDriverStructSet = tFalse;
	m_bIDsJuryStructSet = tFalse;

	continueAfterSection=true;
	SetPropertyBool(PROP_CONTINUE,continueAfterSection);
	SetPropertyBool(PROP_CONTINUE NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_CONTINUE NSSUBPROP_DESCRIPTION, "Allow continous execution of maneuvers.");

}

SR_DriverModule::~SR_DriverModule()
{
}

tResult SR_DriverModule::createTimer()
{
	LOG_INFO(cString::Format("SR_DriverModule::createTimer"));
	// creates timer with 0.5 sec
	__synchronized_obj(m_oCriticalSectionTimerSetup);
	// additional check necessary because input jury structs can be mixed up because every signal is sent three times
	if (m_hTimer == NULL)
	{
		m_hTimer = _kernel->TimerCreate(tTimeStamp(0.5*1000000), 0, static_cast<IRunnable*>(this),
										NULL, NULL, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));
	}
	else
	{
		LOG_ERROR("Timer is already running. Unable to create a new one.");
	}
	RETURN_NOERROR;
}

tResult SR_DriverModule::destroyTimer(IException **__exception_ptr)
{
	__synchronized_obj(m_oCriticalSectionTimerSetup);
	//destroy timer
	if (m_hTimer != NULL)
	{
		tResult nResult = _kernel->TimerDestroy(m_hTimer);
		if (IS_FAILED(nResult))
		{
			LOG_ERROR("Unable to destroy the timer.");
			THROW_ERROR(nResult);
		}
		m_hTimer = NULL;
	}
	//check if handle for some unknown reason still exists
	else
	{
		LOG_WARNING("Timer handle not set, but I should destroy the timer. Try to find a timer with my name.");
		tHandle hFoundHandle = _kernel->FindHandle(adtf_util::cString::Format("%s.timer", OIGetInstanceName()));
		if (hFoundHandle)
		{
			tResult nResult = _kernel->TimerDestroy(hFoundHandle);
			if (IS_FAILED(nResult))
			{
				LOG_ERROR("Unable to destroy the found timer.");
				THROW_ERROR(nResult);
			}
		}
	}

	RETURN_NOERROR;
}

tResult SR_DriverModule::Init(cFilter::tInitStage eStage, IException **__exception_ptr)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	// pins need to be created at StageFirst
	if (eStage == StageFirst)    {

		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,
											 IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
											 (tVoid**)&pDescManager,
											 __exception_ptr));

		// input jury struct
		tChar const * strDesc1 = pDescManager->GetMediaDescription("tJuryStruct");
		RETURN_IF_POINTER_NULL(strDesc1);
		cObjectPtr<IMediaType> pType1 = new cMediaType(0, 0, 0, "tJuryStruct", strDesc1, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(m_JuryStructInputPin.Create("Jury_Struct", pType1, this));
		RETURN_IF_FAILED(RegisterPin(&m_JuryStructInputPin));
		RETURN_IF_FAILED(pType1->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescJuryStruct));

		// input maneuver list
		tChar const * strDesc3 = pDescManager->GetMediaDescription("tManeuverList");
		RETURN_IF_POINTER_NULL(strDesc3);
		cObjectPtr<IMediaType> pType3 = new cMediaType(0, 0, 0, "tManeuverList", strDesc3, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(m_ManeuverListInputPin.Create("Maneuver_List", pType3, this));
		RETURN_IF_FAILED(RegisterPin(&m_ManeuverListInputPin));
		RETURN_IF_FAILED(pType3->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescManeuverList));

		// output driver struct
		tChar const * strDesc2 = pDescManager->GetMediaDescription("tDriverStruct");
		RETURN_IF_POINTER_NULL(strDesc2);
		cObjectPtr<IMediaType> pType2 = new cMediaType(0, 0, 0, "tDriverStruct", strDesc2, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(m_DriverStructOutputPin.Create("Driver_Struct", pType2, this));
		RETURN_IF_FAILED(RegisterPin(&m_DriverStructOutputPin));
		RETURN_IF_FAILED(pType2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescDriverStruct));

		// output general start
		REGISTER_MEDIA_PIN(inPinManeuverComplete, MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_UINT8, "Maneuver_Response");
		INIT_WRAPPED_PIN(outPinRestartSection, "Restart_Section");

		REGISTER_MEDIA_PIN(m_ActiveCommandOutputPin, MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_UINT8, "Maneuver");

	}
	else if(eStage == StageNormal)
	{
		// no ids were set so far
		m_bIDsJuryStructSet = tFalse;
		m_bIDsDriverStructSet = tFalse;

		myGlobalFinishedSection=false;
		LOG_INFO("SR_DriverModule::Init");
		lastTimeStampRun=_clock->GetTime();
		myActiveManeuver=0;
		myActiveSector=-1;
		sendStateError=false;
		sendStateReady=false;
		sendStateRunning=false;
		sendStateComplete=false;
		sendStateStartup=false;

	}
	RETURN_NOERROR;
}

tResult SR_DriverModule::Start(__exception)
{
	myLastEntryID=-1;
	RETURN_IF_FAILED(cFilter::Start(__exception_ptr));
	LOG_INFO(cString::Format("SR_DriverModule::Start"));
	createTimer();
	RETURN_NOERROR;
}

tResult SR_DriverModule::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
	//Reset myLastEntry to allow reading any new command
	//myLastEntryID=-1;
	bool doSendState=true;
	tInt16 tempVal=myLastEntryID;
	stateCar tempStateCar=stateCar_STARTUP;
	if (sendStateError)
	{
		sendStateError=false;
		tempStateCar=stateCar_ERROR;
	}
	else if (sendStateReady)
	{
		sendStateReady=false;
		tempStateCar=stateCar_READY;
		tempVal=sendReadyEntryID;
	}
	else if (sendStateComplete)
	{
		sendStateComplete=false;
		tempStateCar=stateCar_COMPLETE;
	}
	else if (sendStateRunning)
	{
		sendStateRunning=false;
		tempStateCar=stateCar_RUNNING;
	}
	else if (sendStateStartup)
	{
		sendStateStartup=false;
		tempStateCar=stateCar_STARTUP;
	}
	else
	{
		doSendState=false;
	}
	if (doSendState)
	{
		SendState(tempStateCar,tempVal);
	}


	RETURN_NOERROR;


}

tResult SR_DriverModule::SendState(stateCar state, tInt16 i16ManeuverEntry)
{
	__synchronized_obj(m_oCriticalSectionTransmit);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescDriverStruct->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	tInt8 bValue = tInt8(state);
	RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
	{   // focus for sample write lock
		__adtf_sample_write_lock_mediadescription(m_pDescDriverStruct,pMediaSample,pCoder);
		// get the IDs for the items in the media sample
		if(!m_bIDsDriverStructSet)
		{
			pCoder->GetID("i8StateID", m_szIDDriverStructI8StateID);
			pCoder->GetID("i16ManeuverEntry", m_szIDDriverStructI16ManeuverEntry);
			m_bIDsDriverStructSet = tTrue;
		}
		pCoder->Set(m_szIDDriverStructI8StateID, (tVoid*)&bValue);
		pCoder->Set(m_szIDDriverStructI16ManeuverEntry, (tVoid*)&i16ManeuverEntry);
	}
	pMediaSample->SetTime(_clock->GetStreamTime());
	m_DriverStructOutputPin.Transmit(pMediaSample);
	bool m_bDebugModeEnabled=true;
	//debug output to console
	if(m_bDebugModeEnabled)
	{
		switch (state)
		{
		case stateCar_ERROR:
			if(m_bDebugModeEnabled) LOG_INFO(cString::Format("State Controller Module: Send state: ERROR, Maneuver ID %d",i16ManeuverEntry));
			break;
		case stateCar_READY:
			if(m_bDebugModeEnabled) LOG_INFO(cString::Format("State Controller Module: Send state: READY, Maneuver ID %d",i16ManeuverEntry));
			break;
		case stateCar_RUNNING:
			if(m_bDebugModeEnabled) LOG_INFO(cString::Format("State Controller Module: Send state: RUNNING, Maneuver ID %d",i16ManeuverEntry));
			break;
		case stateCar_COMPLETE:
			if(m_bDebugModeEnabled) LOG_INFO(cString::Format("State Controller Module: Send state: COMPLETE, Maneuver ID %d",i16ManeuverEntry));
			break;
		case stateCar_STARTUP:
			if(m_bDebugModeEnabled) LOG_INFO(cString::Format("State Controller Module: Send state: STARTUP, Maneuver ID %d",i16ManeuverEntry));
			break;
		}
	}
	RETURN_NOERROR;
}

tResult SR_DriverModule::Stop(__exception)
{
	RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));
	LOG_INFO(cString::Format("SR_DriverModule::Stop"));
	destroyTimer();
	RETURN_NOERROR;
}

tResult SR_DriverModule::Shutdown(tInitStage eStage, __exception)
{ 
	RETURN_IF_FAILED(cFilter::Shutdown(eStage, __exception_ptr));
	RETURN_NOERROR;
}

tResult SR_DriverModule::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);



	if (nEventCode == IPinEventSink::PE_MediaSampleReceived  )
	{
		LOG_INFO("PinEvent");
		if (pSource==&inPinManeuverComplete)
		{
			uint8_t val = static_cast<uint8_t>(ManeuverResponseError);
			READ_MEDIA_PIN(pMediaSample, uint8_t, val);
			ProcessManeuverComplete(static_cast<ManeuverResponse>(val));

		}
		if (pSource == &m_JuryStructInputPin && m_pDescJuryStruct != NULL)
		{
			tInt8 i8ActionID = -2;
			tInt16 i16entry = -1;

			{   // focus for sample read lock
				__adtf_sample_read_lock_mediadescription(m_pDescJuryStruct,pMediaSample,pCoder);
				// get the IDs for the items in the media sample
				if(!m_bIDsJuryStructSet)
				{
					pCoder->GetID("i8ActionID", m_szIDJuryStructI8ActionID);
					pCoder->GetID("i16ManeuverEntry", m_szIDJuryStructI16ManeuverEntry);
					m_bIDsJuryStructSet = tTrue;
				}

				pCoder->Get(m_szIDJuryStructI8ActionID, (tVoid*)&i8ActionID);
				pCoder->Get(m_szIDJuryStructI16ManeuverEntry, (tVoid*)&i16entry);
			}

			switch (juryActions(i8ActionID))
			{
			case action_GETREADY:
				logger.GLogTrap("RequestReady;ID;"+cString::FromInt(i16entry),_clock->GetTime(),cDateTime::GetCurrentDateTime());
				ReadyReceived(i16entry);
				break;
			case action_START:
				logger.GLogTrap("RequestRun;ID;"+cString::FromInt(i16entry),_clock->GetTime(),cDateTime::GetCurrentDateTime());
				SEND_WRAPPED_SAMPLE(outPinRestartSection,true,_clock->GetStreamTime());
				RunReceived(i16entry);
				break;
			case action_STOP:
				logger.GLogTrap("RequestStop;ID;"+cString::FromInt(i16entry),_clock->GetTime(),cDateTime::GetCurrentDateTime());
				SEND_WRAPPED_SAMPLE(outPinRestartSection,true,_clock->GetStreamTime());
				sendStateError=true;
				StopReceived(i16entry);
				break;
			}

		}
		else if (pSource == &m_ManeuverListInputPin && m_pDescManeuverList != NULL)
		{

			{   // focus for sample read lock
				__adtf_sample_read_lock_mediadescription(m_pDescManeuverList,pMediaSample,pCoder);


				std::vector<tSize> vecDynamicIDs;

				// retrieve number of elements by providing NULL as first paramter
				tSize szBufferSize = 0;
				if(IS_OK(pCoder->GetDynamicBufferIDs(NULL, szBufferSize)))
				{
					// create a buffer depending on the size element
					tChar* pcBuffer = new tChar[szBufferSize];
					vecDynamicIDs.resize(szBufferSize);
					// get the dynamic ids (we already got the first "static" size element)
					if (IS_OK(pCoder->GetDynamicBufferIDs(&(vecDynamicIDs.front()), szBufferSize)))
					{
						// iterate over all elements
						for (tUInt32 nIdx = 0; nIdx < vecDynamicIDs.size(); ++nIdx)
						{
							// get the value and put it into the buffer
							pCoder->Get(vecDynamicIDs[nIdx], (tVoid*)&pcBuffer[nIdx]);
						}

						// set the resulting char buffer to the string object
						m_strManeuverFileString = (const tChar*) pcBuffer;
						LoadManeuverList();
					}

					// cleanup the buffer
					delete pcBuffer;
				}

			}

		}
	}

	RETURN_NOERROR;

}
tResult SR_DriverModule::OnSendState(stateCar stateID, tInt16 i16ManeuverEntry)
{            
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescDriverStruct->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();

	tInt8 value = tInt8(stateID);

	RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
	{   // focus for sample write lock
		__adtf_sample_write_lock_mediadescription(m_pDescDriverStruct,pMediaSample,pCoder);
		// get the IDs for the items in the media sample
		if(!m_bIDsDriverStructSet)
		{
			pCoder->GetID("i8StateID", m_szIDDriverStructI8StateID);
			pCoder->GetID("i16ManeuverEntry", m_szIDDriverStructI16ManeuverEntry);
			m_bIDsDriverStructSet = tTrue;
		}


		pCoder->Set(m_szIDDriverStructI8StateID, (tVoid*)&value);
		pCoder->Set(m_szIDDriverStructI16ManeuverEntry, (tVoid*)&i16ManeuverEntry);
	}

	pMediaSample->SetTime(_clock->GetStreamTime());
	m_DriverStructOutputPin.Transmit(pMediaSample);

	switch (stateID)
	{
	case stateCar_READY:
		LOG_INFO(cString::Format("Driver Module: Send state: READY, Maneuver ID %d",i16ManeuverEntry));
		break;
	case stateCar_RUNNING:
		LOG_INFO(cString::Format("Driver Module: Send state: RUNNING, Maneuver ID %d",i16ManeuverEntry));
		break;
	case stateCar_COMPLETE:
		LOG_INFO(cString::Format("Driver Module: Send state: COMPLETE, Maneuver ID %d",i16ManeuverEntry));
		break;
	case stateCar_ERROR:
		LOG_INFO(cString::Format("Driver Module: Send state: ERROR, Maneuver ID %d",i16ManeuverEntry));
		break;
	case stateCar_STARTUP:
		break;
	}

	RETURN_NOERROR;
}

tResult SR_DriverModule::LoadManeuverList()
{
	LOG_INFO("LoadManeuverList");
	//Reset myLastEntry to allow reading any new command
	myLastEntryID=-1;
	m_sectorList.clear();
	// create dom from string received from pin
	cDOM oDOM;
	oDOM.FromString(m_strManeuverFileString);
	cDOMElementRefList oSectorElems;
	cDOMElementRefList oManeuverElems;

	//read first Sector Elem
	if(IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
	{
		//iterate through sectors
		for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
		{
			//if sector found
			tSector sector;
			sector.id = (*itSectorElem)->GetAttributeUInt32("id");

			if(IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
			{
				//iterate through maneuvers
				for(cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
				{
					tAADC_Maneuver man;
					man.id = (*itManeuverElem)->GetAttributeUInt32("id");
					man.action = (*itManeuverElem)->GetAttribute("action");
					sector.maneuverList.push_back(man);
				}
			}

			m_sectorList.push_back(sector);
		}
	}
	if (oSectorElems.size() > 0)
	{
		LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
		int k=0;
		for (int i=0;i<m_sectorList.size();i++)
		{
			for (int j=0;j<m_sectorList[i].maneuverList.size();j++)
			{
				k++;
				logger.log()<<k<<";"<<cString::Format(m_sectorList[i].maneuverList.at(j).action.GetPtr()).GetPtr()<<endl;

			}
		}
	}
	else
	{
		LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
		RETURN_ERROR(ERR_INVALID_FILE);
	}


	RETURN_NOERROR;
}

tResult SR_DriverModule::ReadyReceived(int entryId)
{
	//New data has arrivedID
	//LOG_INFO(cString::Format("SR_DriverModule::ReadyReceived.",entryId));
	int temp1,temp2;
	if (getSectorAndIndexForID(entryId,temp1,temp2))
	{
		sendStateReady=true;
		sendReadyEntryID=entryId;

	}
	else
	{
		LOG_INFO("Could not find entry ID in maneuver list");
		sendStateError=true;
	}
	RETURN_NOERROR;
}

tResult SR_DriverModule::RunReceived(int entryId)
{
	//LOG_INFO(cString::Format("SR_DriverModule::RunReceived entryID %d",entryId));
	tTimeStamp myTimeStamp=_clock->GetTime();
	tFloat64 myDiff=(myTimeStamp-lastTimeStampRun)/1000000;
	lastTimeStampRun=myTimeStamp;
	//LOG_INFO("Timestamp difference:"+cString::FromFloat64(myDiff));
	//logger.GLogTrap("RunReceived;Diff;"+cString::FromFloat64(myDiff),_clock->GetTime(),cDateTime::GetCurrentDateTime());
	if (myDiff>3.0)//only accept new command if at least some time passes between
	{
		//New data has arrivedID
		LOG_INFO(cString::Format("SR_DriverModule::RunReceived interpreting entryID %d because enough time passed since previous command.",entryId));
		if (getSectorAndIndexForID(entryId,myActiveSector,myActiveManeuver))
		{
			myLastEntryID=entryId;
			ProcessActiveManeuver();

		}
		else
		{
			LOG_INFO("Could not find entry ID in maneuver list");
			sendStateError=true;
		}
	}
	else
	{
		logger.GLogTrap("RejectedRunReceived;Diff;"+cString::FromFloat64(myDiff),_clock->GetTime(),cDateTime::GetCurrentDateTime());
	}
	RETURN_NOERROR;
}

tResult SR_DriverModule::StopReceived(int entryId)
{
	LOG_INFO("SR_DriverModule::StopReceived");
	SendManeuverComplete(0);
	RETURN_NOERROR;
}

cString SR_DriverModule::GetActionStringForActiveManeuver()
{
	//myLogger.GLogTrap("Open_myActiveManeuver"+cString::FromInt(myActiveManeuver),
	//                  _clock->GetTime(),cDateTime::GetCurrentDateTime());
	if (myActiveSector>=0)
	{
		if (myActiveManeuver<m_sectorList[myActiveSector].maneuverList.size())
		{
			return cString::Format(m_sectorList[myActiveSector].maneuverList.at(myActiveManeuver).action.GetPtr()).GetPtr();
		}
		else
		{
			logger.GLogTrap("Fail_Open_myActiveManeuver",
							_clock->GetTime(),cDateTime::GetCurrentDateTime());
			return "";
		}
	}
	else
	{
		return "";
	}
}

ManeuverType SR_DriverModule::GetLocalManeuverForActiveManeuverString(cString vString)
{

	ManeuverType result=ManeuverIdle;
	if (vString.IsEqual("left"))
	{
		result=ManeuverLeft;
	}
	else if (vString.IsEqual("right"))
	{
		result=ManeuverRight;
	}
	else if (vString.IsEqual("straight"))
	{
		result=ManeuverStraight;
	}
	else if (vString.IsEqual("cross_parking"))
	{
		logger.log()<<"Found cross parking in GetLocalManeuverForActiveManeuverString"<<endl;
		result=ManeuverCrossParking;
	}
	else if (vString.IsEqual("pull_out_right"))
	{
		result=ManeuverPulloutRight;
	}
	else if (vString.IsEqual("parallel_parking"))
	{
		result=ManeuverParallelParking;
	}
	else if (vString.IsEqual("pull_out_left"))
	{
		result=ManeuverPulloutLeft;
	}
	return result;
}

tResult SR_DriverModule::ProcessActiveManeuver()
{

	cString myActionString=GetActionStringForActiveManeuver();
	logger.GLogTrap("ProcessActiveManeuver;myActiveManeuver"+
					cString::FromInt(myActiveManeuver)+"=["+
					myActionString+"] in list.",
					_clock->GetTime(),cDateTime::GetCurrentDateTime());
	ManeuverType tempManeuver=GetLocalManeuverForActiveManeuverString(myActionString);
	if (tempManeuver==ManeuverIdle)
	{
		LOG_INFO("Error;Could not make local maneuver for given string");
		sendStateError=true;
	}
	else
	{
		sendStateRunning=true;
		logger.log()<<"Sending out maneuver ["<<tempManeuver<<"]"<<endl;
		uint8_t value=static_cast<uint8_t>(tempManeuver);
		SEND_MEDIA_SAMPLE(m_ActiveCommandOutputPin, uint8_t, value, _clock->GetStreamTime());
		RETURN_NOERROR;
	}


	RETURN_NOERROR;

}

tResult SR_DriverModule::ProcessManeuverComplete(ManeuverResponse vData)
{
	logger.log()<<"ProcessManeuverComplete;"<<vData<<endl;
	if (vData==ManeuverResponseError)
	{
		LOG_INFO("Send back error to driver struct!");
		sendStateError=true;
	}
	else if (vData==ManeuverResponseComplete)
	{
		sendStateComplete=true;
		LOG_INFO("Send back complete to driver struct!");
		//check for end
		if (checkReachedEndOfParcour(myLastEntryID+1))
		{
			LOG_INFO("Parcour end reached!");
			SendManeuverComplete(1);
			//
		}
		else if (checkReachedEndOfSection(myLastEntryID+1))
		{
			LOG_INFO("Section end reached!");
			if (continueAfterSection)
			{
				LOG_INFO("Continuing after section.");
				myLastEntryID++;
				if (getSectorAndIndexForID(myLastEntryID,myActiveSector,myActiveManeuver))
				{
					ProcessActiveManeuver();
				}
				else
				{
					LOG_INFO("ERROR2 on continue sector; Could not find entry ID in maneuver list,maneuver list changed in the meantime?");
					SendManeuverComplete(4);
				}
			}
			else
			{
				LOG_INFO("Stopping car after section.");
				SendManeuverComplete(2);
			}
		}
		else
		{
			LOG_INFO("Still on the list reached!");
			myLastEntryID++;
			if (getSectorAndIndexForID(myLastEntryID,myActiveSector,myActiveManeuver))
			{
				ProcessActiveManeuver();
			}
			else
			{
				LOG_INFO("ERROR; Not reached end of parcour/section but could not find entry ID in maneuver list,maneuver list changed in the meantime?");
				SendManeuverComplete(3);
			}
		}
	}
	RETURN_NOERROR;
}

tResult SR_DriverModule::WriteOutManeuver(int vData)
{
	myLastManeuver=vData;
	//SEND_WRAPPED_SAMPLE(m_ActiveCommandOutputPin,vData,_clock->GetStreamTime());
	RETURN_NOERROR;
}


tResult SR_DriverModule::WriteBoolToPin(WrappedBVOutputPin &vOutputPin, bool vData)
{
	SEND_WRAPPED_SAMPLE(vOutputPin,vData,_clock->GetStreamTime());
	RETURN_NOERROR;
}

tResult SR_DriverModule::SendManeuverStop()
{
	ManeuverType tempManeuver=ManeuverIdle;
	uint8_t value=static_cast<uint8_t>(tempManeuver);
	SEND_MEDIA_SAMPLE(m_ActiveCommandOutputPin, uint8_t, value, _clock->GetStreamTime());
	RETURN_NOERROR;
}

tResult SR_DriverModule::SendManeuverComplete(int vReason)
{
	logger.log()<<"SendManeuverComplete;Reason;"<<vReason<<endl;
	ManeuverType tempManeuver=ManeuverComplete;
	uint8_t value=static_cast<uint8_t>(tempManeuver);
	SEND_MEDIA_SAMPLE(m_ActiveCommandOutputPin, uint8_t, value, _clock->GetStreamTime());
	RETURN_NOERROR;
}

bool SR_DriverModule::checkReachedEndOfParcour(int vEntryID)
{
	int k=0;
	for(unsigned int i = 0; i < m_sectorList.size(); i++)
	{
		for(unsigned int j = 0; j < m_sectorList[i].maneuverList.size(); j++)
		{
			k++;
		}
	}
	if (k==0) {
		return false;
	}
	return vEntryID==k;

}

bool SR_DriverModule::checkReachedEndOfSection(int vEntryID)
{
	if (myActiveSector<m_sectorList.size() && myActiveSector>=0) {
		return m_sectorList[myActiveSector].maneuverList.size()==vEntryID;
	}
	return false;
}


bool SR_DriverModule::getSectorAndIndexForID(int vEntryID, int &vSector, int &vIndex)
{
	for(unsigned int i = 0; i < m_sectorList.size(); i++)
	{
		for(unsigned int j = 0; j < m_sectorList[i].maneuverList.size(); j++)
		{
			if(m_sectorList[i].maneuverList.at(j).id==vEntryID)
			{
				//set active variables to match found location
				myActiveSector=i;
				myActiveManeuver=j;
				return true;
			}
		}
	}
	return false;
}

tResult SR_DriverModule::PropertyChanged(const tChar *strName)
{

	if (cString::IsEqual(strName, PROP_CONTINUE))
	{
		continueAfterSection = GetPropertyBool(PROP_CONTINUE);
	}
}




