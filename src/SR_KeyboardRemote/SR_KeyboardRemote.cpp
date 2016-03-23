#include "SR_KeyboardRemote.h"

using namespace adtf;

#define PROP_CYCLETIME "Timing::Timer control cycle [msec]"
#define PROP_DEBUG1 "Debug::Activate debug1 output"
#define PROP_FILE "File::Keyboard commands file"
//REMOTE
//Steering properties
#define PROP_INCREMENTAL_STEERING "Remote::Steering::Incremental acceleration [m/s]"
#define PROP_INC_STEER_RETURN_IDLE "Remote::Steering::Incremental return to idle steer [servo angle]"
#define PROP_IDLE_STEER "Remote::Steering::Straight position of wheels [servo angle]"
//Speed properties
#define PROP_INCREMENTAL_ACCELERATION "Remote::Speed::Incremental acceleration [m/s]"
#define PROP_INC_SPEED_RETURN_IDLE "Remote::Speed::Incremental return to idle speed [m/s]"
#define PROP_MAX_SPEED_FWD "Remote::Speed::Maximum forward speed [m/s]"
#define PROP_MAX_SPEED_BACK "Remote::Speed::Maximum backward speed [m/s]"
#define PROP_IDLE_SPEED "Remote::Speed::Neutral speed command [m/s]"
#define PROP_WATCHDOG_TIME "Remote::Watchdog [s]"

ADTF_FILTER_PLUGIN("SpaceRacer Keyboard Remote", OID_SR_KEYBOARD_REMOTE, SR_KeyboardRemote)

SR_KeyboardRemote::SR_KeyboardRemote(const tChar *info) : cFilter(info),m_hTimer(NULL),
	myLogger(OID_SR_KEYBOARD_REMOTE), myDebug1(false)
{

	SetPropertyStr(PROP_FILE,"/home/aadc/myKeyboardRemote.txt");
	SetPropertyBool(PROP_FILE NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr(PROP_FILE NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "TXT Files (*.txt)");
	SetPropertyStr(PROP_FILE NSSUBPROP_DESCRIPTION, "Here you have to set the file static maneuvers");

	SetPropertyInt(PROP_CYCLETIME, 50);
	SetPropertyBool(PROP_CYCLETIME NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_CYCLETIME NSSUBPROP_DESCRIPTION, "The resolution for the local sequence timer.");

	pPROP_WATCHDOG_TIME=5;
	SetPropertyInt(PROP_WATCHDOG_TIME, pPROP_WATCHDOG_TIME);
	SetPropertyBool(PROP_WATCHDOG_TIME NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_WATCHDOG_TIME NSSUBPROP_DESCRIPTION, "The watchdog to send idle speed/steering after no commands received on remote.");

	pPROP_INCREMENTAL_ACCELERATION=0.4;
	SetPropertyFloat(PROP_INCREMENTAL_ACCELERATION, pPROP_INCREMENTAL_ACCELERATION);
	SetPropertyBool(PROP_INCREMENTAL_ACCELERATION NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_INCREMENTAL_ACCELERATION NSSUBPROP_DESCRIPTION, "Per control cycle: When pressing forward/backward, computed speed will be adjusted by this amount.");

	pPROP_INCREMENTAL_STEERING=10;
	SetPropertyFloat(PROP_INCREMENTAL_STEERING, pPROP_INCREMENTAL_STEERING);
	SetPropertyBool(PROP_INCREMENTAL_STEERING NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_INCREMENTAL_STEERING NSSUBPROP_DESCRIPTION, "Per control cycle: Computed steer will be adjusted by this amount towards applied left/right command.");

	pPROP_IDLE_STEER=90.0;
	SetPropertyFloat(PROP_IDLE_STEER, pPROP_IDLE_STEER);
	SetPropertyBool(PROP_IDLE_STEER NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_IDLE_STEER NSSUBPROP_DESCRIPTION, "Neutral steering angle.");

	pPROP_IDLE_SPEED=0.0;
	SetPropertyFloat(PROP_IDLE_SPEED, pPROP_IDLE_SPEED);
	SetPropertyBool(PROP_IDLE_SPEED NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_IDLE_SPEED NSSUBPROP_DESCRIPTION, "Neutral velocity.");

	pPROP_INC_STEER_RETURN_IDLE=10;
	SetPropertyFloat(PROP_INC_STEER_RETURN_IDLE, pPROP_INC_STEER_RETURN_IDLE);
	SetPropertyBool(PROP_INC_STEER_RETURN_IDLE NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_INC_STEER_RETURN_IDLE NSSUBPROP_DESCRIPTION, "Per control cycle: Computed steer will be adjusted by this amount towards idle when there is no left/right command applied.");

	pPROP_INC_SPEED_RETURN_IDLE=0.3;
	SetPropertyFloat(PROP_INC_SPEED_RETURN_IDLE, pPROP_INC_SPEED_RETURN_IDLE);
	SetPropertyBool(PROP_INC_SPEED_RETURN_IDLE NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_INC_SPEED_RETURN_IDLE NSSUBPROP_DESCRIPTION, "Per control cycle: Computed speed will be adjusted by this amount towards idle when there is no forward/backward command applied.");

	pPROP_MAX_SPEED_FWD=1.0;
	SetPropertyFloat(PROP_MAX_SPEED_FWD, pPROP_MAX_SPEED_FWD);
	SetPropertyBool(PROP_MAX_SPEED_FWD NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_MAX_SPEED_FWD NSSUBPROP_DESCRIPTION, "Maximum positive computed speed.");

	pPROP_MAX_SPEED_BACK=-1.0;
	SetPropertyFloat(PROP_MAX_SPEED_BACK, pPROP_MAX_SPEED_BACK);
	SetPropertyBool(PROP_MAX_SPEED_BACK NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_MAX_SPEED_BACK NSSUBPROP_DESCRIPTION, "Maximum negative computed speed.");

	SetPropertyBool(PROP_DEBUG1,myDebug1);
	SetPropertyBool(PROP_DEBUG1 NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_DEBUG1 NSSUBPROP_DESCRIPTION, "Enabling debugging output.");

}

SR_KeyboardRemote::~SR_KeyboardRemote()
{
}

tResult SR_KeyboardRemote::Init(cFilter::tInitStage eStage, IException **__exception_ptr)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst) {
		INIT_WRAPPED_PIN(qSteeringAngle, "Steering_Angle");
		INIT_WRAPPED_PIN(qSpeed, "qSpeed");
		INIT_WRAPPED_PIN(qDataAvailable, "Data_Available");
		INIT_WRAPPED_PIN(outPinCommand, "Command");
		INIT_WRAPPED_PIN(qPinF, "F");
		INIT_WRAPPED_PIN(qPinG, "G");
		INIT_WRAPPED_PIN(qPinH, "H");
		INIT_WRAPPED_PIN(qPinJ, "J");
		INIT_WRAPPED_PIN(qPinK, "K");


	} else if (eStage == StageNormal) {
		myDebug1 = GetPropertyBool(PROP_DEBUG1);
		LOG_INFO("My debug1 is ["+cString::FromBool(myDebug1)+"]");
		myLogger.GLogTrap("SR_KeyboardRemote::Init",_clock->GetTime(),cDateTime::GetCurrentDateTime());
		myComputedSpeed=pPROP_IDLE_SPEED;
		myComputedSteer=pPROP_IDLE_STEER;
		myLeft=false;
		myRight=false;
		myForward=false;
		myBackward=false;
		myBrake=false;
		myKeyboardCommands.resize(10);
		wL_myKeyboardCommands.resize(10);

	}

	RETURN_NOERROR;
}

tResult SR_KeyboardRemote::OnPinEvent(IPin *source, tInt eventCode, tInt nParam1, tInt nParam2, IMediaSample *mediaSample)
{
	RETURN_IF_POINTER_NULL(source);
	RETURN_IF_POINTER_NULL(mediaSample);
	if (eventCode==IPinEventSink::PE_MediaSampleReceived)
	{


	}
	RETURN_NOERROR;
}

tResult SR_KeyboardRemote::WriteOutMotionAndData(float vSpeed, float vSteer, bool vDataReady)
{
	SEND_WRAPPED_SAMPLE(qSpeed,vSpeed,_clock->GetStreamTime());
	SEND_WRAPPED_SAMPLE(qSteeringAngle,vSteer,_clock->GetStreamTime());
	SEND_WRAPPED_SAMPLE(qDataAvailable,vDataReady,_clock->GetStreamTime());
	RETURN_NOERROR;
}

tResult SR_KeyboardRemote::createTimer()
{

	myLastWatchdogTriggered=false;
	bool myDataValid=true;

	if (myDataValid)
	{
		// creates timer with 0.5 sec
		__synchronized_obj(m_oCriticalSectionTimerSetup);
		// additional check necessary because input jury structs can be mixed up because every signal is sent three times
		if (m_hTimer == NULL)
		{
			adtf_util::cString myTimerName(OIGetInstanceName());
			myTimerName.Append(".timer");
			LOG_INFO("Generating timer with string: ["+myTimerName+"] and cycle time ["+cString::FromInt(GetPropertyInt(PROP_CYCLETIME))+"]");

			m_hTimer = _kernel->TimerCreate(tTimeStamp(GetPropertyInt(PROP_CYCLETIME)*1000), 0, static_cast<IRunnable*>(this),
											NULL, NULL, 0, 0, myTimerName);


		}
		else
		{
			;//LOG_ERROR("Timer is already running. Unable to create a new one.");
		}
	}
	else
	{
		LOG_ERROR("Data was not valid. Timer not created.");
	}
	RETURN_NOERROR;
}

tResult SR_KeyboardRemote::destroyTimer(__exception)
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
	RETURN_NOERROR;
}


tResult SR_KeyboardRemote::Run(tInt nActivationType, const tVoid *pvUserData, tInt nUserDataSize, IException **__exception_ptr)
{

	tTimeStamp InputTimeStamp;
	InputTimeStamp=_clock->GetTime();
	tInt64 myLocalTime=(tInt64)InputTimeStamp;

	tInt64 myTimeElapsed=myLocalTime-myPreviousLocalTime;
	if (myTimeElapsed<0 || myTimeElapsed>(300)*1000)
	{
		myTimeElapsed=0;
	}
	myTimeElapsed/=1000;
	//LOG_INFO("My time elapsed"+cString::FromInt(myTimeElapsed));
	myPreviousLocalTime=myLocalTime;

	cFile myFileHandle;
	cFilename fileCalibration = GetPropertyStr(PROP_FILE);
	int myOutputCmd=-1;
	vector<bool> myLocalLetterCommands;
	myLocalLetterCommands.clear();
	myLocalLetterCommands.resize(LETTER_COMMANDS);
	if (myFileHandle.Open(fileCalibration,cFile::OM_Read))
	{
		cString myTemp;
		myFileHandle.ReadLine(myTemp);
		//LOG_INFO(myTemp);
		myFileHandle.Close();


		//Process data
		cStringList myStringList;
		myTemp.Split(myStringList," ");
		//LOG_INFO(myStringList[0]+";"+myStringList[1]+";"+myStringList[2]+";"+myStringList[3]+";"+myStringList[4]+";"+myStringList[5]);
		if (myStringList.GetItemCount()!=22)
		{
			//LOG_ERROR("Keyboard remote:Failed to read complete data command row, size of list was ["+cString::FromInt(myStringList.GetItemCount())+"]");
			//ResetCommands();
		}
		else
		{
			cString myMotionCommandTemp=myTemp.Left(9);
			if (!myLastCommandLine.IsEqual(myMotionCommandTemp) && !myStartup)
			{
				//LOG_INFO(myTemp);
				//LOG_INFO(myLastCommandLine);
				//new command received on keyboard remote, reset watchdog
				myWatchdog=0;
			}
			myLastCommandLine=myMotionCommandTemp;

			myLeft=(myStringList[0].AsInt())>0;
			myRight=(myStringList[1].AsInt())>0;
			myForward=(myStringList[2].AsInt())>0;
			myBackward=(myStringList[3].AsInt())>0;
			myBrake=(myStringList[4].AsInt())>0;
			for (int i=0;i<DIGIT_COMMANDS+LETTER_COMMANDS;i++)
			{

				myKeyboardCommands[i]=((myStringList[5+i].AsInt())>0);
				/*
				if (i==0)
				{
					LOG_INFO("Checking command ["+cString::FromInt(i)+"] which is now ["+cString::FromBool(myKeyboardCommands[i])+"]");
				}*/
				if (myKeyboardCommands[i] && ! wL_myKeyboardCommands[i])
				{
					if (i<DIGIT_COMMANDS)
					{
						//Rising edge of command, set output;
						//if (i==0)
						//{
						//	myOutputCmd=10;
						//}
						//else
						//{
							myOutputCmd=i;
						//}
					}
					else
					{
						//Rising edge of pin command
						//LOG_INFO("Rising edge on pin ["+cString::FromInt(i-DIGIT_COMMANDS)+"]");
						myLocalLetterCommands[i-DIGIT_COMMANDS]=true;
					}
					//Reset keyboard controls also
					myWatchdog=pPROP_WATCHDOG_TIME*1000;
					//LOG_INFO("Found command ["+cString::FromInt(i)+"]");
				}
				wL_myKeyboardCommands[i]=myKeyboardCommands[i];
			}
		}
	}
	else
	{
		;//LOG_ERROR("Keyboard remote:Failed to open data command file.");
		ResetCommands();
	}
	/*
	LOG_INFO("Left:["+cString::FromBool(myLeft)+"] "+
			 "Right:["+cString::FromBool(myRight)+"] "+
			 "Up:["+cString::FromBool(myForward)+"] "+
			 "Down:["+cString::FromBool(myBackward)+"] "+
			 "Brake:["+cString::FromBool(myBrake)+"]");*/
	myWatchdog+=myTimeElapsed;
	if (myBrake)
	{
		myComputedSpeed=pPROP_IDLE_SPEED;
	}
	else if (myForward)
	{
		myComputedSpeed+=pPROP_INCREMENTAL_ACCELERATION;
	}
	else if (myBackward)
	{
		myComputedSpeed-=pPROP_INCREMENTAL_ACCELERATION;
	}
	else
	{
		//Degrade the velocity towards idle
		float newValue;
		if (myComputedSpeed>pPROP_IDLE_SPEED)
		{
			newValue=myComputedSpeed-pPROP_INC_SPEED_RETURN_IDLE;
			if (newValue<pPROP_IDLE_SPEED)
			{
				myComputedSpeed=pPROP_IDLE_SPEED;
			}
			else
			{
				myComputedSpeed=newValue;
			}
		}
		else if (myComputedSpeed<pPROP_IDLE_SPEED)
		{
			newValue=myComputedSpeed+pPROP_INC_SPEED_RETURN_IDLE;
			if (newValue>pPROP_IDLE_SPEED)
			{
				myComputedSpeed=pPROP_IDLE_SPEED;
			}
			else
			{
				myComputedSpeed=newValue;
			}
		}
	}
	if (myLeft)
	{
		myComputedSteer-=pPROP_INCREMENTAL_STEERING;
		if (myComputedSteer<STEERING_MAX_LEFT)
		{
			myComputedSteer=STEERING_MAX_LEFT;
		}
	}
	else if (myRight)
	{
		myComputedSteer+=pPROP_INCREMENTAL_STEERING;
		if (myComputedSteer>STEERING_MAX_RIGHT)
		{
			myComputedSteer=STEERING_MAX_RIGHT;
		}
	}
	else
	{
		if (myComputedSteer>pPROP_IDLE_STEER)
		{
			myComputedSteer-=pPROP_INC_STEER_RETURN_IDLE;
			if (myComputedSteer<pPROP_IDLE_STEER)
			{
				myComputedSteer=pPROP_IDLE_STEER;
			}
		}
		else if (myComputedSteer<pPROP_IDLE_STEER)
		{
			myComputedSteer+=pPROP_INC_STEER_RETURN_IDLE;
			if (myComputedSteer>pPROP_IDLE_STEER)
			{
				myComputedSteer=pPROP_IDLE_STEER;
			}
		}

	}
	if (myComputedSpeed>pPROP_MAX_SPEED_FWD)
	{
		myComputedSpeed=pPROP_MAX_SPEED_FWD;
	}
	else if (myComputedSpeed<pPROP_MAX_SPEED_BACK)
	{
		myComputedSpeed=pPROP_MAX_SPEED_BACK;
	}
	//LOG_INFO("Computed speed: ["+cString::FromFloat64(myComputedSpeed)+"] "+
	//         "Computed steer: ["+cString::FromFloat64(myComputedSteer)+"]");
	if (myWatchdog>pPROP_WATCHDOG_TIME*1000)
	{
		myWatchdog=pPROP_WATCHDOG_TIME*1000;
		if (!myLastWatchdogTriggered)
		{
			;//LOG_ERROR("Keyboard remote: Remote watchdog failed. Issue new command every "+cString::FromInt(pPROP_WATCHDOG_TIME)+" seconds to prevent trigger.");
		}
		//watchdog has responded, send false to data available and idle speed/steering
		WriteOutMotionAndData(pPROP_IDLE_SPEED,pPROP_IDLE_STEER,false);

		myLastWatchdogTriggered=true;
	}
	else
	{
		if (myLastWatchdogTriggered)
		{
			;//LOG_INFO("Keyboard remote: Remote watchdog reset. System operational now.");
		}
		//system is healthy
		WriteOutMotionAndData(myComputedSpeed,myComputedSteer,true);
		myLastWatchdogTriggered=false;
	}
	myStartup=false;
	if (myOutputCmd>=0)
	{
		;//LOG_INFO("Sending output command ["+cString::FromInt(myOutputCmd)+"]");
		SEND_WRAPPED_SAMPLE(outPinCommand,myOutputCmd,_clock->GetTime());
	}
	for (int i=0;i<LETTER_COMMANDS;i++)
	{
		if (myLocalLetterCommands[i])
		{
			myLocalLetterCommands[i]=false;
			if (i==0)
			{
				SEND_WRAPPED_SAMPLE(qPinF,true,_clock->GetTime());
			}
			if (i==1)
			{
				SEND_WRAPPED_SAMPLE(qPinG,true,_clock->GetTime());
			}
			if (i==2)
			{
				SEND_WRAPPED_SAMPLE(qPinH,true,_clock->GetTime());
				SetLowSpeedLimits();
			}
			if (i==3)
			{
				SEND_WRAPPED_SAMPLE(qPinJ,true,_clock->GetTime());
				SetMediumSpeedLimits();
			}
			if (i==4)
			{
				SEND_WRAPPED_SAMPLE(qPinK,true,_clock->GetTime());
				SetHighSpeedLimits();
			}

		}
	}
	RETURN_NOERROR;
}

tResult SR_KeyboardRemote::Stop(IException **__exception_ptr)
{
	__synchronized_obj(m_oCriticalSectionTimerSetup);
	destroyTimer(__exception_ptr);
	return cFilter::Stop(__exception_ptr);
}



tResult SR_KeyboardRemote::Start(IException **__exception_ptr)
{
	myWatchdog=pPROP_WATCHDOG_TIME*1000+1;//trigger watchdog at beginning
	myStartup=true;
	for (int i=0;i<10;i++)
	{
		wL_myKeyboardCommands[i]=false;
	}
	createTimer();
	RETURN_NOERROR;
}

void SR_KeyboardRemote::ResetCommands()
{
	myLeft=false;
	myRight=false;
	myForward=false;
	myBackward=false;
	myBrake=false;
}

void SR_KeyboardRemote::SetHighSpeedLimits()
{
	pPROP_MAX_SPEED_FWD=2.5;
	pPROP_MAX_SPEED_BACK=-2.5;
}

void SR_KeyboardRemote::SetMediumSpeedLimits()
{
	pPROP_MAX_SPEED_FWD=1;
	pPROP_MAX_SPEED_BACK=-1;
}

void SR_KeyboardRemote::SetLowSpeedLimits()
{
	pPROP_MAX_SPEED_FWD=0.5;
	pPROP_MAX_SPEED_BACK=-0.5;
}

tResult SR_KeyboardRemote::PropertyChanged(const tChar *strName)
{

	if (cString::IsEqual(strName, PROP_INCREMENTAL_STEERING))
	{
		pPROP_INCREMENTAL_STEERING = GetPropertyFloat(PROP_INCREMENTAL_STEERING, pPROP_INCREMENTAL_STEERING);
	}
	if (cString::IsEqual(strName, PROP_INC_STEER_RETURN_IDLE))
	{
		pPROP_INC_STEER_RETURN_IDLE = pPROP_INC_STEER_RETURN_IDLE;
	}
	if (cString::IsEqual(strName, PROP_IDLE_STEER))
	{
		pPROP_IDLE_STEER = pPROP_IDLE_STEER;
	}
	if (cString::IsEqual(strName, PROP_INCREMENTAL_ACCELERATION))
	{
		pPROP_INCREMENTAL_ACCELERATION = pPROP_INCREMENTAL_ACCELERATION;
	}
	if (cString::IsEqual(strName, PROP_MAX_SPEED_FWD))
	{
		pPROP_MAX_SPEED_FWD = GetPropertyFloat(PROP_MAX_SPEED_FWD, pPROP_MAX_SPEED_FWD);
	}
	if (cString::IsEqual(strName, PROP_MAX_SPEED_BACK))
	{
		pPROP_MAX_SPEED_BACK = GetPropertyFloat(PROP_MAX_SPEED_BACK, pPROP_MAX_SPEED_BACK);
	}
	if (cString::IsEqual(strName, PROP_IDLE_SPEED))
	{
		pPROP_IDLE_SPEED = pPROP_IDLE_SPEED;
	}
	if (cString::IsEqual(strName, PROP_WATCHDOG_TIME))
	{
		pPROP_WATCHDOG_TIME = pPROP_WATCHDOG_TIME;
	}
	RETURN_NOERROR;
}


