#include "SR_Controller.h"

using namespace adtf;
using namespace adtf_graphics;


#define PROP_CYCLETIME "Timing::Timer control cycle [msec]"
#define PROP_DEBUG1 "Debug::Activate debug1 output"
#define PROP_FILE "File::Settings file"
#define PROP_FILE_XML "XML::Configuration File For Interpolation"
#define PROP_XML_PRINT_INITIAL "XML::Print initial table to Console"

#define LOG_US_VAL false
#define US_FILTER_SIZE 8




#define DEBUG_PARALLEL_PARKING false

ADTF_FILTER_PLUGIN("SpaceRacer Speed Controller", OID_SR_CONTROLLER, SR_Controller)

SR_Controller::SR_Controller(const tChar *info) : cFilter(info),m_hTimer(NULL),
	logger(OID_SR_CONTROLLER),parallelParkingController(this),pullOutParallelParkingController(this),crossParkingController(this),pullOutCrossParkingController(this),pullOutLeftCrossParkingController(this)
{

	SetPropertyStr(PROP_FILE,"../../../config_files/speed_controller/speedController2.txt");
	SetPropertyBool(PROP_FILE NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr(PROP_FILE NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "TXT Files (*.txt)");
	SetPropertyStr(PROP_FILE NSSUBPROP_DESCRIPTION, "Here you have to set the controller settings");

	SetPropertyInt(PROP_CYCLETIME, 50);
	SetPropertyBool(PROP_CYCLETIME NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_CYCLETIME NSSUBPROP_DESCRIPTION, "The control step for the local sequence timer.");

	myDebug1=false;
	SetPropertyBool(PROP_DEBUG1,myDebug1);
	SetPropertyBool(PROP_DEBUG1 NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_DEBUG1 NSSUBPROP_DESCRIPTION, "Enabling debugging output.");

	SetPropertyStr(PROP_FILE_XML,"../../../config_files/speed_controller/SpeedController_LOAD.xml");
	SetPropertyBool(PROP_FILE_XML NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr(PROP_FILE_XML NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
	SetPropertyStr(PROP_FILE_XML NSSUBPROP_DESCRIPTION, "The XML to be loaded has to be set here");

	SetPropertyBool(PROP_XML_PRINT_INITIAL,tFalse);
	SetPropertyStr(PROP_XML_PRINT_INITIAL NSSUBPROP_DESCRIPTION, "If enabled the loaded points of the interpolation table of the XML are printed to console");
}

SR_Controller::~SR_Controller()
{
}

tResult SR_Controller::Init(cFilter::tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst) {

		//INPUT PINS
		INIT_WRAPPED_PIN(inPinRemoteAv, "RemoteKB_Available");
		INIT_WRAPPED_PIN(inPinRemoteKBSetSpeed, "Input_RemoteKB_Set_Speed");
		INIT_WRAPPED_PIN(inPinRemoteKBSetSteering, "Input_RemoteKB_Set_Steer");
		cObjectPtr<IMediaType> inputType;
		RETURN_IF_FAILED(AllocMediaType(&inputType, MEDIA_TYPE_ODOMETRY, MEDIA_SUBTYPE_ODOMETRY_POSE, __exception_ptr));
		RETURN_IF_FAILED(inPinActualPose.Create("ActualPose", inputType, this));
		RETURN_IF_FAILED(RegisterPin(&inPinActualPose));
		RETURN_IF_FAILED(AllocMediaType(&inputType, MEDIA_TYPE_CONTROLLERCOMMAND, MEDIA_SUBTYPE_CONTROLLER_COMMAND, __exception_ptr));
		RETURN_IF_FAILED(inPinCommand.Create("Command", inputType, this));
		RETURN_IF_FAILED(RegisterPin(&inPinCommand));
		//Ultrasonic block
		INIT_WRAPPED_PIN(inPinUltrasonic_Front,"Ultrasonic_Front_Center");
		INIT_WRAPPED_PIN(inPinUltrasonic_Rear,"Ultrasonic_Rear_Center");
		INIT_WRAPPED_PIN(inPinUltrasonic_Front_Center_Left,"Ultrasonic_Front_Center_Left");
		INIT_WRAPPED_PIN(inPinUltrasonic_Front_Center_Right,"Ultrasonic_Front_Center_Right");
		INIT_WRAPPED_PIN(inPinUltrasonic_Front_Left,"Ultrasonic_Front_Left");
		INIT_WRAPPED_PIN(inPinUltrasonic_Front_Right,"Ultrasonic_Front_Right");
		INIT_WRAPPED_PIN(inPinUltrasonic_Rear_Left,"Ultrasonic_Rear_Left");
		INIT_WRAPPED_PIN(inPinUltrasonic_Rear_Right,"Ultrasonic_Rear_Right");
		INIT_WRAPPED_PIN(inPinUltrasonic_Side_Left,"Ultrasonic_Side_Left");
		INIT_WRAPPED_PIN(inPinUltrasonic_Side_Right,"Ultrasonic_Side_Right");

		//OUTPUT PINS
		INIT_WRAPPED_PIN(outPinReachingPose, "ReachingPose");
		INIT_WRAPPED_PIN(outPinSpeed, "XML_Out_Speed");
		INIT_WRAPPED_PIN(outPinSteer, "XML_Out_Steer");
		INIT_WRAPPED_PIN(outPinHeadLights, "HeadLights");
		INIT_WRAPPED_PIN(outPinBrakeLights, "BrakeLights");
		INIT_WRAPPED_PIN(outPinRevLights, "RevLights");





	} else if (eStage == StageNormal) {
		myDebug1 = GetPropertyBool(PROP_DEBUG1);
		LOG_INFO("My debug1 is ["+cString::FromBool(myDebug1)+"]");
		myUS.Filter_Front.clear();
		myUS.Filter_Rear.clear();
		myUS.Filter_FrontLeft.clear();
		myUS.Filter_FrontRight.clear();
		myUS.Filter_RearLeft.clear();
		myUS.Filter_FrontCenterLeft.clear();
		myUS.Filter_FrontCenterRight.clear();
		myUS.Filter_RearRight.clear();
		myUS.Filter_SideLeft.clear();
		myUS.Filter_SideRight.clear();

		IntErrorSteeringCounter = 0;
		IntErrorSteeringVector.clear();
		myUS.Counter_Front=0;
		myUS.Counter_Rear=0;
		myUS.Counter_FrontLeft=0;
		myUS.Counter_FrontRight=0;
		myUS.Counter_FrontCenterLeft=0;
		myUS.Counter_FrontCenterRight=0;
		myUS.Counter_RearLeft=0;
		myUS.Counter_RearRight=0;
		myUS.Counter_SideLeft=0;
		myUS.Counter_SideRight=0;
		lastCommand.type=DesiredPose;
		resetIBuffer=false;
	} else if(eStage == StageGraphReady)
	{
		;


	}

	RETURN_NOERROR;
}

tResult SR_Controller::OnPinEvent(IPin *source, tInt eventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
	RETURN_IF_POINTER_NULL(source);

	__synchronized_obj(m_critSecOnPinEvent);
	if (eventCode==IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL)
	{
		RETURN_IF_POINTER_NULL(pMediaSample);
		bool theValue;float theFloatValue;
		tUInt32 timeStamp;
		if (source==inPinRemoteAv.ppin)
		{

			RETURN_IF_FAILED(static_cast<WrappedBVInputPin *>(&inPinRemoteAv)->getValue(pMediaSample, &theValue, &timeStamp));
			myRemoteAvailable = theValue;
		}
		if (source==&inPinActualPose)
		{
			ProcessPose(pMediaSample,actualPose,"ActualPose",false);
			myResetActualPoseTimer=true;
		}
		if (source==&inPinCommand)
		{
			ProcessCommand(pMediaSample,myCommand,"DesiredPose",true);
			//LOG_INFO("Received Desired Pose");
			//double a=GetDistanceBetweenOdos(myActualPose,myDesiredPose);
			//myLogger.GLogTrap("a;"+cString::FromFloat64(static_cast<float>(a)));


		}
		if(source == inPinUltrasonic_Front.ppin){
			myUS.Dist_Front=ProcessUS("US FRONT",inPinUltrasonic_Front,pMediaSample,myUS.Counter_Front,myUS.Filter_Front,LOG_US_VAL);
		}
		if(source == inPinUltrasonic_Rear.ppin){
			myUS.Dist_Rear=ProcessUS("US REAR",inPinUltrasonic_Rear,pMediaSample,myUS.Counter_Rear,myUS.Filter_Rear,LOG_US_VAL);
			//LOG_INFO("US REAR ["+cString::FromFloat64(myUS.Dist_Rear)+"]");
		}
		if(source == inPinUltrasonic_Rear_Left.ppin){
			myUS.Dist_RearLeft=ProcessUS("US REAR LEFT",inPinUltrasonic_Rear_Left,pMediaSample,myUS.Counter_RearLeft,myUS.Filter_RearLeft,LOG_US_VAL);
		}
		if(source == inPinUltrasonic_Rear_Right.ppin){
			myUS.Dist_RearRight=ProcessUS("US REAR RIGHT",inPinUltrasonic_Rear_Right,pMediaSample,myUS.Counter_RearRight,myUS.Filter_RearRight,LOG_US_VAL);
		}
		if(source == inPinUltrasonic_Front_Left.ppin){
			myUS.Dist_FrontLeft=ProcessUS("US FRONT LEFT",inPinUltrasonic_Front_Left,pMediaSample,myUS.Counter_FrontLeft,myUS.Filter_FrontLeft,LOG_US_VAL);
		}
		if(source == inPinUltrasonic_Front_Right.ppin){
			myUS.Dist_FrontRight=ProcessUS("US FRONT RIGHT",inPinUltrasonic_Front_Right,pMediaSample,myUS.Counter_FrontRight,myUS.Filter_FrontRight,LOG_US_VAL);
		}
		if(source == inPinUltrasonic_Front_Center_Left.ppin){
			myUS.Dist_FrontCenterLeft=ProcessUS("US FRONT CENTER LEFT",inPinUltrasonic_Front_Center_Left,pMediaSample,myUS.Counter_FrontCenterLeft,myUS.Filter_FrontCenterLeft,LOG_US_VAL);
		}
		if(source == inPinUltrasonic_Front_Center_Right.ppin){
			myUS.Dist_FrontCenterRight=ProcessUS("US FRONT CENTER RIGHT",inPinUltrasonic_Front_Center_Right,pMediaSample,myUS.Counter_FrontCenterRight,myUS.Filter_FrontCenterRight,LOG_US_VAL);
		}
		if(source == inPinUltrasonic_Side_Left.ppin){
			myUS.Dist_SideLeft=ProcessUS("US SIDE LEFT",inPinUltrasonic_Side_Left,pMediaSample,myUS.Counter_SideLeft,myUS.Filter_SideLeft,LOG_US_VAL);
		}
		if(source == inPinUltrasonic_Side_Right.ppin){
			myUS.Dist_SideRight=ProcessUS("US SIDE RIGHT",inPinUltrasonic_Side_Right,pMediaSample,myUS.Counter_SideRight,myUS.Filter_SideRight,LOG_US_VAL);
		}
		if (source==&inPinRemoteKBSetSpeed.pin)
		{
			inPinRemoteKBSetSpeed.getValue(pMediaSample,&theFloatValue);
			myRemoteSpeedSetpoint=theFloatValue;//myREAD_FLOAT32_VALUE_FROM_PIN(&inPinRemoteKBSetSpeed,mediaDescRemoteKBSetSpeed,pMediaSample,bufIDGetRemoteKBSetSpeed,bufIDRemoteKBSetSpeed);
		}
		if (source==&inPinRemoteKBSetSteering.pin)
		{
			inPinRemoteKBSetSteering.getValue(pMediaSample,&theFloatValue);//myREAD_FLOAT32_VALUE_FROM_PIN(&inPinRemoteKBSetSpeed,mediaDescRemoteKBSetSpeed,pMediaSample,bufIDGetRemoteKBSetSpeed,bufIDRemoteKBSetSpeed);
			myRemoteSteerSetpoint=theFloatValue;
			//LOG_INFO("Remote Steer:"+cString::FromFloat64(myRemoteSteerSetpoint));
		}

	}
	RETURN_NOERROR;
}

float SR_Controller::GetDeccDist()
{
	return (actualPose.speed*actualPose.speed)/(2.0*Decceleration);
}

tResult SR_Controller::WriteOutXMLSpeed(tFloat32 vSpeed)
{
	if (myWinkSteringArduinoReady)
	{
		myWinkSteringArduinoReady--;
		vSpeed=83.0;
	}
	SEND_WRAPPED_SAMPLE(outPinSpeed,vSpeed,_clock->GetTime());
	//RETURN_IF_FAILED(outSpeed.setValue() myWRITE_FLOAT32_VALUE_TO_PIN(vSpeed,&m_oOutputActuator,m_pDescActuator,m_bOutputActuatorGetID,m_buIDActuatorF32Value));
	RETURN_NOERROR;
}

tResult SR_Controller::WriteOutXMLSteer(tFloat32 vData)
{
	SEND_WRAPPED_SAMPLE(outPinSteer,vData,_clock->GetTime());
	//RETURN_IF_FAILED(outSpeed.setValue() myWRITE_FLOAT32_VALUE_TO_PIN(vSpeed,&m_oOutputActuator,m_pDescActuator,m_bOutputActuatorGetID,m_buIDActuatorF32Value));
	RETURN_NOERROR;
}

bool SR_Controller::CheckUSFrontAll()
{
	return myUS.Dist_Front<set.pSTOP_US_DIST_FRONT || myUS.Dist_FrontLeft<set.pSTOP_US_DIST_FRONT || myUS.Dist_FrontRight<set.pSTOP_US_DIST_FRONT ||
			myUS.Dist_FrontCenterLeft<set.pSTOP_US_DIST_FRONT || myUS.Dist_FrontCenterRight<set.pSTOP_US_DIST_FRONT;
}

bool SR_Controller::CheckUSFrontAll(float value)
{
	return	myUS.Dist_Front<value ||
			myUS.Dist_FrontLeft<value ||
			myUS.Dist_FrontRight<value ||
			myUS.Dist_FrontCenterLeft<value ||
			myUS.Dist_FrontCenterRight<value;
}

bool SR_Controller::CheckUSFrontCenterLeftRight(float value)
{
	return	myUS.Dist_Front<value ||
			myUS.Dist_FrontCenterLeft<value ||
			myUS.Dist_FrontCenterRight<value;
}

bool SR_Controller::CheckUSFrontCenterOnly(float value)
{
	return	myUS.Dist_Front<value;
}

bool SR_Controller::CheckUSRear()
{
	return myUS.Dist_Rear<set.pSTOP_US_DIST_REAR ||
			myUS.Dist_RearLeft<set.pSTOP_US_DIST_REAR ||
			myUS.Dist_RearRight<set.pSTOP_US_DIST_REAR;
}

bool SR_Controller::CheckUSRear(float value)
{
	return  myUS.Dist_Rear<value ||
			myUS.Dist_RearLeft<value ||
			myUS.Dist_RearRight<value;
}

bool SR_Controller::CheckUSRearCenterOnly(float value)
{
	return  myUS.Dist_Rear<value;
}

bool SR_Controller::CheckUSNothingInFront()
{
	return myUS.Dist_Front>set.pNOTHING_US_FRONT && myUS.Dist_FrontLeft>set.pNOTHING_US_FRONT && myUS.Dist_FrontRight>set.pNOTHING_US_FRONT &&
			myUS.Dist_FrontCenterLeft>set.pNOTHING_US_FRONT && myUS.Dist_FrontCenterRight>set.pNOTHING_US_FRONT;
}

bool SR_Controller::CheckUSNothingInRear()
{
	return myUS.Dist_Rear<set.pNOTHING_US_REAR || myUS.Dist_RearLeft<set.pNOTHING_US_REAR || myUS.Dist_RearRight<set.pNOTHING_US_REAR;
}

float SR_Controller::CheckEstop(float desiredVelocity)
{
	//float slowdownspeed=0.2;
	float stopwindow=0.2;
	float speedStep=0.01;
	float deccDist=GetDeccDist();
	float obstacleDist;
	bool slowdown=false;
	bool stop=false;
	int sign=-1;
	if (desiredVelocity>0)
	{
		slowdown=CheckUSFrontCenterLeftRight(deccDist+stopwindow);
		stop=CheckUSFrontCenterLeftRight(stopwindow);
		sign=1;
		obstacleDist=myUS.Dist_Front;
	}
	else if (desiredVelocity<0)
	{
		slowdown=CheckUSRearCenterOnly(deccDist+0.4);
		stop=CheckUSRearCenterOnly(0.4);
		obstacleDist=myUS.Dist_Rear;
	}
	else
	{
		return 0;
	}
	if (obstacleDist>1.5)
	{
		obstacleDist=1.5;
	}
	if (stop || slowdown)
	{
		return 0;
	}
	else
	{
		if (actualPose.speed+speedStep>desiredVelocity || obstacleDist>(deccDist+stopwindow))
		{
			return desiredVelocity;
		}
		else
		{
			return actualPose.speed+sign*speedStep;
		}
	}
	return 0;
}


tResult SR_Controller::createTimer()
{
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
			if (myDebug1) LOG_INFO("Generating timer with string: ["+myTimerName+"] and cycle time ["+cString::FromInt(GetPropertyInt(PROP_CYCLETIME))+"]");
			//LOG_INFO("SR_Controller: ON");
			m_hTimer = _kernel->TimerCreate(tTimeStamp(GetPropertyInt(PROP_CYCLETIME)*1000), 0, static_cast<IRunnable*>(this),
											NULL, NULL, 0, 0, myTimerName);
		}
	}
	RETURN_NOERROR;
}

tResult SR_Controller::destroyTimer(__exception)
{
	//WriteBoolToPin(&ready_to_run,false);
	//WriteBoolToPin(&outHazardLight,false);
	__synchronized_obj(m_oCriticalSectionTimerSetup);
	//destroy timer
	if (m_hTimer != NULL)
	{
		tResult nResult = _kernel->TimerDestroy(m_hTimer);
		//LOG_INFO("SR_Controller: OFF");
		if (IS_FAILED(nResult))
		{
			LOG_ERROR("Unable to destroy the timer.");
			THROW_ERROR(nResult);
		}
		m_hTimer = NULL;
	}
	RETURN_NOERROR;
}

tResult SR_Controller::LoadSettings()
{
	mErrors.clear();
	// reload maneuver file
	cFilename fileCalibration = GetPropertyStr(PROP_FILE);
	ADTF_GET_CONFIG_FILENAME(fileCalibration);
	cFile myFileHandle;
	cString myTemp;
	if (myFileHandle.Open(fileCalibration,cFile::OM_Read))
	{
		while(!myFileHandle.IsEof())
		{
			myFileHandle.ReadLine(myTemp);
			if (myDebug1) {LOG_INFO("Read line=["+myTemp+"]");}
			if (myTemp.IsEmpty())
			{
				RETURN_ERROR(ERR_INVALID_FILE);
			}
			else if (myTemp.StartsWith("#"))
			{
				;//comment, ignore
			}
			else if (myTemp.StartsWith("@"))
			{
				;//parameter value, type @,P_GAIN,0.005
				cStringList myStringList;
				myTemp.Split(myStringList,",");
				if (myStringList.GetItemCount()!=3)
				{
					LOG_ERROR("Invalid data in file, every row should be styled @,%s,%f");
				}
				else
				{
					if (myStringList[1].IsEqual("P_GAIN"))
					{
						if (myDebug1) {LOG_INFO("Loaded P_GAIN=["+myStringList[2]+"]");}
						P_Gain=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("I_GAIN"))
					{
						if (myDebug1) {LOG_INFO("Loaded I_GAIN=["+myStringList[2]+"]");}
						I_Gain=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("P_GAIN_S"))
					{
						if (myDebug1) {LOG_INFO("Loaded P_GAIN_S=["+myStringList[2]+"]");}
						P_Gain_S=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("I_GAIN_S"))
					{
						if (myDebug1) {LOG_INFO("Loaded I_GAIN_S=["+myStringList[2]+"]");}
						I_Gain_S=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("I_BUFFER_S"))
					{
						if (myDebug1) {LOG_INFO("Loaded I_BUFFER_S=["+myStringList[2]+"]");}
						I_BufferSSize=myStringList[2].AsInt();
					}
					if (myStringList[1].IsEqual("I_BUFFER"))
					{
						if (myDebug1) {LOG_INFO("Loaded I_BUFFER=["+myStringList[2]+"]");}
						myI_BufferSize=myStringList[2].AsInt();
					}
					if (myStringList[1].IsEqual("OUT_MIN"))
					{
						if (myDebug1) {LOG_INFO("Loaded OUT_MIN=["+myStringList[2]+"]");}
						myMinimumCmd=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("OUT_MAX"))
					{
						if (myDebug1) {LOG_INFO("Loaded OUT_MAX=["+myStringList[2]+"]");}
						myMaximumCmd=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("IDLE_STEER"))
					{
						if (myDebug1) {LOG_INFO("Loaded IDLE_STEER=["+myStringList[2]+"]");}
						myIdleSteerXML=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("IDLE_SPEED"))
					{
						if (myDebug1) {LOG_INFO("Loaded IDLE_SPEED=["+myStringList[2]+"]");}
						myIdleSpeed=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("IDLE_SPEED_XML"))
					{
						if (myDebug1) {LOG_INFO("Loaded IDLE_SPEED_XML=["+myStringList[2]+"]");}
						myIdleSpeedXML=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("STANDSTILL_WINDOW"))
					{
						if (myDebug1) {LOG_INFO("Loaded STANDSTILL_WINDOW=["+myStringList[2]+"]");}
						myStandstillWindow=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("DEFAULT_CRUISE_VELOCITY"))
					{
						if (myDebug1) {LOG_INFO("Loaded DEFAULT_CRUISE_VELOCITY=["+myStringList[2]+"]");}
						DefaultCruiseVelocity=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("POSE_REACHED"))
					{
						if (myDebug1) {LOG_INFO("Loaded POSE_REACHED=["+myStringList[2]+"]");}
						PoseReached=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("DECCELERATION"))
					{
						if (myDebug1) {LOG_INFO("Loaded DECCELERATION=["+myStringList[2]+"]");}
						Decceleration=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("TIMER_POSE"))
					{
						if (myDebug1) {LOG_INFO("Loaded TIMER_POSE=["+myStringList[2]+"]");}
						tWAIT_FOR_NEW_POSE=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("POSE_REACHED_BEFORE"))
					{
						if (myDebug1) {LOG_INFO("Loaded POSE_REACHED_BEFORE=["+myStringList[2]+"]");}
						tPOSE_REACHED_BEFORE=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("POSE_FAIL_HYSTERESIS"))
					{
						if (myDebug1) {LOG_INFO("Loaded POSE_FAIL_HYSTERESIS=["+myStringList[2]+"]");}
						tPOSE_FAIL_HYSTERESIS=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("SAME_ORIENTATION_DEG"))
					{
						if (myDebug1) {LOG_INFO("Loaded SAME_ORIENTATION_DEG=["+myStringList[2]+"]");}
						tSAME_ORIENTATION_DEG=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("SAME_ORIENTATION_Y"))
					{
						if (myDebug1) {LOG_INFO("Loaded SAME_ORIENTATION_Y=["+myStringList[2]+"]");}
						tSAME_ORIENTATION_Y=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("POSE_FAIL_PARKING_HYSTERESIS"))
					{
						if (myDebug1) {LOG_INFO("Loaded POSE_FAIL_PARKING_HYSTERESIS=["+myStringList[2]+"]");}
						tPOSE_FAIL_PARKING_HYSTERESIS=myStringList[2].AsFloat64();
					}

					if (myStringList[1].IsEqual("STOP_US_DIST_FRONT"))
					{
						if (myDebug1) {LOG_INFO("Loaded STOP_US_DIST_FRONT=["+myStringList[2]+"]");}
						set.pSTOP_US_DIST_FRONT=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("STOP_US_DIST_REAR"))
					{
						if (myDebug1) {LOG_INFO("Loaded STOP_US_DIST_REAR=["+myStringList[2]+"]");}
						set.pSTOP_US_DIST_REAR=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("MIN_VEL_US"))
					{
						if (myDebug1) {LOG_INFO("Loaded MIN_VEL_US=["+myStringList[2]+"]");}
						set.pMIN_VEL_US=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("INHIBIT_US_FRONT"))
					{
						if (myDebug1) {LOG_INFO("Loaded INHIBIT_US_FRONT=["+myStringList[2]+"]");}
						set.pINHIBIT_US_FRONT=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("INHIBIT_US_REAR"))
					{
						if (myDebug1) {LOG_INFO("Loaded INHIBIT_US_REAR=["+myStringList[2]+"]");}
						set.pINHIBIT_US_REAR=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("NOTHING_US_FRONT"))
					{
						if (myDebug1) {LOG_INFO("Loaded NOTHING_US_FRONT=["+myStringList[2]+"]");}
						set.pNOTHING_US_FRONT=myStringList[2].AsFloat64();
					}
					if (myStringList[1].IsEqual("NOTHING_US_REAR"))
					{
						if (myDebug1) {LOG_INFO("Loaded NOTHING_US_REAR=["+myStringList[2]+"]");}
						set.pNOTHING_US_REAR=myStringList[2].AsFloat64();
					}


				}
			}
		}
		myFileHandle.Close();

	}
	else
	{
		LOG_ERROR("Settings file not found");
	}
	RETURN_NOERROR;

}


tResult SR_Controller::Run(tInt nActivationType, const tVoid *pvUserData, tInt nUserDataSize, IException **__exception_ptr)
{

	mySystemActive=ControllerCheckData();
	float myXMLOutputSpeed=90.0,myXMLOutputAngle=90.0;
	float myDesiredVelocity=0,myDesiredSteering=0;
	if (mySystemActive)
	{
		if (myRemoteAvailable)
		{
			float setpoint=CheckEstop(myRemoteSpeedSetpoint);
			ProcessBlockVelocity(setpoint,static_cast<float>(actualPose.speed),false,myXMLOutputSpeed);
			ProcessBlockSteering(90-myRemoteSteerSetpoint,false,myXMLOutputAngle);
		}
		else if (myCommand.type==DesiredPose || myCommand.type==DesiredCrossing)
		{

			ProcessBlockPose(actualPose,myCommand.pose,true,myDesiredVelocity,myDesiredSteering);
			float setpoint=CheckEstop(myDesiredVelocity);
			ProcessBlockVelocity(setpoint,static_cast<float>(actualPose.speed),false,myXMLOutputSpeed);
			ProcessBlockSteering(myDesiredSteering,false,myXMLOutputAngle);
		}
		else if (myCommand.type==ParkingParallel)
		{
			parallelParkingController.process(myDesiredVelocity,myDesiredSteering);
			ProcessBlockVelocity(myDesiredVelocity,static_cast<float>(actualPose.speed),false,myXMLOutputSpeed);
			ProcessBlockSteering(myDesiredSteering,false,myXMLOutputAngle);
		}
		else if (myCommand.type==PullOutParallel)
		{
			pullOutParallelParkingController.process(myDesiredVelocity,myDesiredSteering);
			ProcessBlockVelocity(myDesiredVelocity,static_cast<float>(actualPose.speed),false,myXMLOutputSpeed);
			ProcessBlockSteering(myDesiredSteering,false,myXMLOutputAngle);
		}
		else if (myCommand.type==ParkingCross)
		{
			crossParkingController.process(myDesiredVelocity,myDesiredSteering);
			ProcessBlockVelocity(myDesiredVelocity,static_cast<float>(actualPose.speed),false,myXMLOutputSpeed);
			ProcessBlockSteering(myDesiredSteering,false,myXMLOutputAngle);
		}
		else if (myCommand.type==PullOutCrossLeft)
		{
			pullOutLeftCrossParkingController.process(myDesiredVelocity,myDesiredSteering);
			ProcessBlockVelocity(myDesiredVelocity,static_cast<float>(actualPose.speed),false,myXMLOutputSpeed);
			ProcessBlockSteering(myDesiredSteering,false,myXMLOutputAngle);
		}
		else if (myCommand.type==PullOutCrossRight)
		{
			pullOutCrossParkingController.process(myDesiredVelocity,myDesiredSteering);
			ProcessBlockVelocity(myDesiredVelocity,static_cast<float>(actualPose.speed),false,myXMLOutputSpeed);
			ProcessBlockSteering(myDesiredSteering,false,myXMLOutputAngle);
		}
	}
	else
	{
		myXMLOutputSpeed=myIdleSpeedXML;
		myXMLOutputAngle=myIdleSteerXML;
	}
	mySpeedSetpoint=myDesiredVelocity;
	ProcessLights();
	//Set outputs
	WriteOutXMLSpeed(myXMLOutputSpeed);
	WriteOutXMLSteer(myXMLOutputAngle);

	RETURN_NOERROR;
}
tResult SR_Controller::Stop(IException **__exception_ptr)
{
	__synchronized_obj(m_oCriticalSectionTimerSetup);
	destroyTimer(__exception_ptr);
	return cFilter::Stop(__exception_ptr);
}



tResult SR_Controller::Start(IException **__exception_ptr)
{
	wL_myBrakeLights=false;
	wL_myRevLights=false;
	wL_mySystemActive=false;
	myFinishedArduinoInit=false;
	myMeasSpeed=0;
	mySequenceTime=0;
	actualPose.speed=0;
	actualPose.x=0;
	actualPose.y=0;
	actualPose.theta=0;
	myCommand.pose.theta=0;
	myCommand.pose.speed=0;
	myCommand.pose.x=0;
	myCommand.pose.y=0;
	myEstop = false;
	myResetActualPoseTimer=false;
	myResetDesiredPoseTimer=false;
	hasFinishedPoseProc=true;
	myRemoteAvailable=false;
	myHeadLightRequired=false;
	wL_myBrakeLights=false;
	wL_myRevLights=false;
	LoadSettings();//controller settings
	LoadConfigurationData();//XML calibration
	myCruiseVelocity=0;
	myHeadLightRequired=true;
	myHeadLightVal=false;
	myBrakeLightRequired=true;
	myBrakeLightVal=false;
	myRevLightRequired=true;
	myRevLightVal=false;
	myRemoteSteerSetpoint=90.0;
	myOutputSpeed=0.0;
	myRemoteSpeedSetpoint=0.0;
	myActualPoseTimer=0;
	myDesiredPoseTimer=0;
	mySetLast_a=false;
	myWinkSteringArduinoReady=0;
	myUS.Dist_Front=100;
	myUS.Dist_Rear=100;
	myUS.Dist_RearLeft=100;
	myUS.Dist_RearRight=100;
	myUS.Dist_FrontCenterLeft=100;
	myUS.Dist_FrontCenterRight=100;
	myUS.Dist_FrontLeft=100;
	myUS.Dist_FrontRight=100;

	cyclesFrontStop=0;
	cyclesRearStop=0;
	createTimer();
	RETURN_NOERROR;
}

tResult SR_Controller::LoadConfigurationData()
{
	m_xValues.clear();
	m_yValues.clear();
	//Get path of configuration file
	m_fileConfig = GetPropertyStr(PROP_FILE_XML);
	if (myDebug1) {LOG_INFO("LoadConfigurationData");}
	// check if file exits
	if (m_fileConfig.IsEmpty())
	{
		LOG_ERROR("cCalibrationXml: Configuration file not found");
		RETURN_ERROR(ERR_INVALID_FILE);
	}

	// create absolute path
	ADTF_GET_CONFIG_FILENAME(m_fileConfig);
	m_fileConfig = m_fileConfig.CreateAbsolutePath(".");

	//Load file, parse configuration, print the data

	if (cFileSystem::Exists(m_fileConfig))
	{
		cDOM oDOM;
		oDOM.Load(m_fileConfig);
		//load settings for calibration mode
		cDOMElementRefList oElemsSettings;
		if(IS_OK(oDOM.FindNodes("calibration/settings", oElemsSettings)))
		{
			for (cDOMElementRefList::iterator itElem = oElemsSettings.begin(); itElem != oElemsSettings.end(); ++itElem)
			{
				cDOMElement* pConfigElement;
				if (IS_OK((*itElem)->FindNode("mode", pConfigElement)))
				{

					cString rdMode = pConfigElement->GetData();
					if (myDebug1) {LOG_INFO(adtf_util::cString::Format("cCalibrationXml: %s",rdMode.GetPtr()));}
					if (cString::Compare(rdMode,"linear")==0)
						m_iCalibrationMode=1;
					else if (cString::Compare(rdMode,"cubic")==0)
						m_iCalibrationMode=2;
					else if (cString::Compare(rdMode,"none")==0)
						m_iCalibrationMode=3;
				}
			}
		}
		cString rdMode;
		switch (m_iCalibrationMode)
		{
		case 1:
			rdMode = "linear";
			break;
		case 2:
			rdMode = "cubic";
			break;
		case 3:
			rdMode = "none";
			break;
		}
		if (myDebug1) {LOG_INFO(adtf_util::cString::Format("cCalibrationXml: Calibration mode is %s",rdMode.GetPtr()));}
		//load supporting points
		if (m_iCalibrationMode!=3)
		{
			cDOMElementRefList oElems;
			if(IS_OK(oDOM.FindNodes("calibration/supportingPoints/point", oElems)))
			{
				for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
				{

					cDOMElement* pConfigElement;
					if (IS_OK((*itElem)->FindNode("xValue", pConfigElement)))
					{
						m_xValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
					}
					if (IS_OK((*itElem)->FindNode("yValue", pConfigElement)))
					{
						m_yValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
					}
				}
			}
			if (oElems.size() > 0)
			{
				if (GetPropertyBool(PROP_XML_PRINT_INITIAL))
				{
					for (tUInt i = 0; i<m_xValues.size();i++)
					{
						if (i>m_yValues.size()) break;
						LOG_INFO(cString::Format("cCalibrationXml: supportingPoint #%d: (%lf/%lf)",i,m_xValues[i],m_yValues[i]));
					}
				}
			}
			else
			{
				LOG_ERROR("cCalibrationXml: no supporting points in given file found!");
				RETURN_ERROR(ERR_INVALID_FILE);
			}
			//checks if data are valid
			RETURN_IF_FAILED(CheckConfigurationData());
		}
	}
	else
	{
		LOG_ERROR("cCalibrationXml: Configured configuration file not found");
		RETURN_ERROR(ERR_INVALID_FILE);
	}

	RETURN_NOERROR;
}

tResult SR_Controller::CheckConfigurationData()
{
	//checks if the xValues of the calibration table are increasing
	for (vector<tFloat32>::iterator it = m_xValues.begin(); it != m_xValues.end() ; it++)
	{
		vector<tFloat32>::iterator it2 = it;
		it2++;
		if (it2 != m_xValues.end())
		{
			// next values is smaller than current value
			if ((tFloat32(*it) > tFloat32(*it2)))
			{
				LOG_ERROR(cString::Format("cCalibrationXml: The xValues in the file %s are not in increasing order. Please reorder the points!",m_fileConfig.GetPtr()));
				RETURN_ERROR(ERR_INVALID_FILE);
			}
		}
	}

	RETURN_NOERROR;
}

tFloat32 SR_Controller::getLinearInterpolatedValue(tFloat32 fl32InputValue)
{
	// requested value is smaller than smallest value in table
	if (fl32InputValue<m_xValues.front())
	{
		if (m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("cCalibrationXml: requested x-value %f is lower than smallest x-value in calibration table",fl32InputValue));
		return m_yValues.front();
	}
	// requested value is bigger than biggest value in table
	else if (fl32InputValue>m_xValues.back())
	{
		if (m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("cCalibrationXml: requested x-value %f is higher than highes x-value in calibration table",fl32InputValue));
		return m_yValues.back();
	}
	// search in vector for corresponding index (smallest neighbor)
	tUInt iIndex;
	if (m_xValues.size() > 2)
	{
		for (iIndex = 0; iIndex<m_xValues.size();iIndex++)
		{
			if (m_xValues[iIndex]>=fl32InputValue) break;
		}
		// get smaller neighbor
		if (iIndex!=0) iIndex = iIndex -1;
	}
	else iIndex = 0;

	if ((m_xValues[iIndex+1]-m_xValues[iIndex])!=0)
	{
		// doing the linear interpolation
		tFloat32 f32Value = (fl32InputValue-m_xValues[iIndex])*(m_yValues[iIndex+1]-m_yValues[iIndex])/(m_xValues[iIndex+1]-m_xValues[iIndex])+m_yValues[iIndex];

		//tFloat32 security check to send only minimum or maximum value of table
		if (f32Value > *max_element(m_yValues.begin(),m_yValues.end() ))
			return *max_element(m_yValues.begin(),m_yValues.end()) ;
		else if (f32Value < *min_element(m_yValues.begin(),m_yValues.end()) )
			return *min_element(m_yValues.begin(),m_yValues.end()) ;
		else
			return f32Value;
	}
	else
	{
		LOG_ERROR("cCalibrationXml: invalid table in xml!");
		return 0;
	}
}




tResult SR_Controller::ProcessCommand(IMediaSample *pMediaSample, ControllerCommand &vPose, cString vName, bool vLog)
{
	//myActualPose
	ControllerCommand myTempData;
	{
		// Keep the lock for as short as we can
		__sample_read_lock(pMediaSample, ControllerCommand, data);
		myTempData = *data;
		vPose=*data;
		// Lock goes out of scope here
	}
	if (vLog)
	{
		if (myTempData.type==DesiredPose || myCommand.type==DesiredCrossing)
		{
			logger.GLogTrap(vName+";x;"+
							cString::FromFloat64(myTempData.pose.x)+";y;"+
							cString::FromFloat64(myTempData.pose.y)+";theta;"+
							cString::FromFloat64(myTempData.pose.theta)+";v;"+
							cString::FromFloat64(myTempData.pose.speed)+";act.x;"+
							cString::FromFloat64(actualPose.x)+";act.y;"+
							cString::FromFloat64(actualPose.y)+";act.theta;"+
							cString::FromFloat64(actualPose.theta)+";act.v;"+
							cString::FromFloat64(actualPose.speed)+";a;"+
							cString::FromFloat64(GetDistanceBetweenOdos(myTempData.pose,actualPose)));
			myResetDesiredPoseTimer=true;
		}
		else if (myTempData.type==ParkingParallel)\
		{
			parallelParkingController.Init(myTempData.pose,false);
		}
		else if (myTempData.type==PullOutParallel)\
		{
			pullOutParallelParkingController.Init(myTempData.pose,false);
		}
		else if (myTempData.type==ParkingCross)\
		{
			crossParkingController.Init(myTempData.pose,true);
		}
		else if (myTempData.type==PullOutCrossLeft)
		{
			pullOutLeftCrossParkingController.Init(myTempData.pose,false);
		}
		else if (myTempData.type==PullOutCrossRight)\
		{
			pullOutCrossParkingController.Init(myTempData.pose,false);
		}
		if (lastCommand.type==PullOutCrossRight || lastCommand.type==PullOutCrossLeft || lastCommand.type==PullOutParallel) {
			resetIBuffer=true;
		}
		lastCommand = myCommand;

		/*
		else if (myTempData.type==PullOutParallel)\
		{
			logger.log() << "ParkingParallelPullOutCommand;"<<"ref.x;" << myTempData.pose.x << "y;"<< myTempData.pose.y <<"theta;"<< myTempData.pose.theta
													   <<"act.x;" << actualPose.x << "y;"<< actualPose.y <<"theta;"<< actualPose.theta<<endl;


		}
		else if (myTempData.type==ParkingCross)\
		{
			logger.log() << "ParkingCrossCommand;"<<"ref.x;" << myTempData.pose.x << "y;"<< myTempData.pose.y <<"theta;"<< myTempData.pose.theta
													   <<"act.x;" << actualPose.x << "y;"<< actualPose.y <<"theta;"<< actualPose.theta<<endl;


		}
		else if (myTempData.type==PullOutCrossRight)\
		{
			logger.log() << "ParkingCrossPullOutCommand;"<<"ref.x;" << myTempData.pose.x << "y;"<< myTempData.pose.y <<"theta;"<< myTempData.pose.theta
													   <<"act.x;" << actualPose.x << "y;"<< actualPose.y <<"theta;"<< actualPose.theta<<endl;
			//initialize parallel data
			//myParkData.PullOutPState=PullOutCrossState::BeginProcedure;
			myParkData.ReferencePose=myTempData.pose;
			myParkData.AccumulatedTheta=0;
			myParkData.StateOk=true;

		}*/
	}
	RETURN_NOERROR;
}

double SR_Controller::fixAngle360Deg(double a) {
	while (a>180)
	{
		a-=360;
	}
	while (a<-180)
	{
		a+=360;
	}
	return a;
}

OdometryPose SR_Controller::getActualPose() const
{
	return actualPose;
}

tResult SR_Controller::ProcessPose(IMediaSample *pMediaSample, OdometryPose &vPose, cString vName, bool vLog)
{
	//myActualPose
	OdometryPose myTempData;
	{
		// Keep the lock for as short as we can
		__sample_read_lock(pMediaSample, OdometryPose, data);
		myTempData = *data;
		vPose=*data;
		// Lock goes out of scope here
	}
	if (vLog)
	{
		logger.GLogTrap(vName+";x;"+
						cString::FromFloat64(myTempData.x)+";y;"+
						cString::FromFloat64(myTempData.y)+";theta;"+
						cString::FromFloat64(myTempData.theta)+";v;"+
						cString::FromFloat64(myTempData.speed)+";act.x;"+
						cString::FromFloat64(actualPose.x)+";act.y;"+
						cString::FromFloat64(actualPose.y)+";act.theta;"+
						cString::FromFloat64(actualPose.theta)+";act.v;"+
						cString::FromFloat64(actualPose.speed)+";a;"+
						cString::FromFloat64(GetDistanceBetweenOdos(myTempData,actualPose)));
	}
	RETURN_NOERROR;
}

bool SR_Controller::ControllerCheckData()
{
	/*
	bool somethingInFront=CheckUSFront();
	if (somethingInFront)
	{
		LOG_INFO("Something in front!");
	}
	bool somethingInRear=CheckUSRear();
	if (somethingInRear)
	{
		LOG_INFO("Something in rear!");
	}
	*/
	if (cyclesFrontStop>0)
	{
		cyclesFrontStop--;
	}
	if (cyclesRearStop>0)
	{
		cyclesRearStop--;
	}
	tTimeStamp InputTimeStamp;
	InputTimeStamp=_clock->GetTime();
	tInt64 myLocalTime=(tInt64)InputTimeStamp;

	tInt64 myTimeElapsed=myLocalTime-myPreviousLocalTime;
	if (myTimeElapsed<0 || myTimeElapsed>(300)*1000)
	{
		logger.GLogTrap("TimeElapsedFault;"+cString::FromInt64(myTimeElapsed),_clock->GetTime(),cDateTime::GetCurrentDateTime());
		myTimeElapsed=0;
	}

	//Condition for running the speed controller loop
	myPreviousLocalTime=myLocalTime;

	//Apply ratio to obtain msec
	myTimeElapsed/=1000;
	mySequenceTime+=myTimeElapsed;

	if (mySequenceTime>6000 && !myFinishedArduinoInit)
	{
		myFinishedArduinoInit=true;
		myWinkSteringArduinoReady=5;
	}

	if (myResetActualPoseTimer)
	{
		myResetActualPoseTimer=false;

		myActualPoseTimer=tWAIT_FOR_NEW_POSE;
	}
	else
	{
		myActualPoseTimer-=myTimeElapsed;
	}
	if (myResetDesiredPoseTimer)
	{
		myResetDesiredPoseTimer=false;
		hasFinishedPoseProc=false;
		mySetLast_a=false;
		myCruiseVelocity=myCommand.cruiseVelocity;
		myDesiredPoseTimer=tWAIT_FOR_NEW_POSE;
	}
	else
	{
		myDesiredPoseTimer-=myTimeElapsed;
	}



	if (myDesiredPoseTimer<0)
	{
		myDesiredPoseTimer=-1;
		//No new desired pose found in watchdog
	}
	if (myActualPoseTimer<0)
	{
		myActualPoseTimer=-1;
		//No new actual pose
	}
	bool HigherLevelProcOk=false;
	if (myCommand.type==ParkingParallel)
	{
		HigherLevelProcOk=parallelParkingController.getStateOk();
	}
	else if (myCommand.type==ParkingCross)
	{
		HigherLevelProcOk=crossParkingController.getStateOk();
	}
	else if (myCommand.type==PullOutParallel)
	{
		HigherLevelProcOk=pullOutParallelParkingController.getStateOk();
	}
	else if (myCommand.type==PullOutCrossLeft)
	{
		HigherLevelProcOk=pullOutLeftCrossParkingController.getStateOk();
	}
	else if (myCommand.type==PullOutCrossRight)
	{
		HigherLevelProcOk=pullOutCrossParkingController.getStateOk();
	}
	else if (myCommand.type==DesiredPose || myCommand.type==DesiredCrossing)
	{
		HigherLevelProcOk=myDesiredPoseTimer>0;
	}
	//Common condition: odometry continuous
	HigherLevelProcOk = HigherLevelProcOk && myActualPoseTimer>0;

	if (myRemoteAvailable)
	{
		if (parallelParkingController.getStateOk()) {parallelParkingController.resetState(SR_ParallelParkingController::Ready);}
		if (crossParkingController.getStateOk()) {crossParkingController.resetState(SR_CrossParkingController::Ready);}
		if (pullOutCrossParkingController.getStateOk()) {pullOutCrossParkingController.resetState(SR_PullOutCrossParkingController::Ready);}
		if (pullOutParallelParkingController.getStateOk()) {pullOutParallelParkingController.resetState(SR_PullOutParallelParkingController::Ready);}
		return true;
	}
	else if (HigherLevelProcOk)
	{
		return true;
	}
	else
	{
		return false;
	}
}



tResult SR_Controller::ProcessBlockPose(OdometryPose vActualPose, OdometryPose vDesiredPose,bool vLog, float &qDesiredVelocity, float &qDesiredSteering)
{
	qDesiredVelocity=0;
	double b,x,y;
	double a=GetDistanceBetweenOdos(vActualPose,vDesiredPose);

	if (!mySetLast_a)
	{
		/*
		myLogger.GLogTrap("Reset;a;"+cString::FromFloat64(static_cast<float>(a))+
						  ";last_a;"+cString::FromFloat64(static_cast<float>(myLast_a)),
						  _clock->GetTime(),cDateTime::GetCurrentDateTime());
		*/
		myLast_a=a;
		mySetLast_a=true;

	}
	//Steering controller always runnning
	b=atan2(vDesiredPose.y-vActualPose.y,vDesiredPose.x-vActualPose.x)-cStdMath::MATH_DEG2RAD*vDesiredPose.theta;
	x=a*sin(b);
	y=a*cos(b);

	float myDeltaTheta=fixAngle360Deg(vDesiredPose.theta-vActualPose.theta);
	float limitedDeltaTheta;
	if (myDeltaTheta>30)
	{
		limitedDeltaTheta=30;
	}
	else if (myDeltaTheta<-30)
	{
		limitedDeltaTheta=-30;
	}
	else
	{
		limitedDeltaTheta=myDeltaTheta;
	}
	float myGainTerm=P_Gain_S*cStdMath::MATH_RAD2DEG*atan2(x,fabs(myCruiseVelocity));


	//Filter relative velocity
	if (resetIBuffer){
		IntErrorSteeringVector.clear();
		IntErrorSteeringCounter=0;
		resetIBuffer=false;
	}
	if (IntErrorSteeringCounter < I_BufferSSize) {
		IntErrorSteeringVector.push_back(atan2(x,fabs(myCruiseVelocity)));
	}
	else
	{
		IntErrorSteeringVector[IntErrorSteeringCounter % I_BufferSSize]=atan2(x,fabs(myCruiseVelocity));
	}
	float IntErrorSteering = 0;
	for (size_t i = 0; i < IntErrorSteeringVector.size(); ++i) {
		IntErrorSteering += IntErrorSteeringVector[i];
	}
	IntErrorSteeringCounter++;

	float myIntTerm=I_Gain_S*IntErrorSteering;
	if (myDeltaTheta>15 && myCommand.type==DesiredCrossing){
		qDesiredSteering=30;
	}
	else if (myDeltaTheta<-15 && myCommand.type==DesiredCrossing){
		qDesiredSteering=-30;
	}
	else{
		qDesiredSteering=limitedDeltaTheta+myGainTerm+myIntTerm;
	}
	if (myCruiseVelocity<0)
	{
		qDesiredSteering*=-1;
	}

	int reason=0;
	bool WritePoseReached=false;
	//Speed controller checking for reached pose
	float myHysteresis;
	myHysteresis=tPOSE_FAIL_HYSTERESIS;
	if ((a>(myLast_a+myHysteresis)) && mySetLast_a && !hasFinishedPoseProc)
	{
		//Car is going further from goal
		hasFinishedPoseProc=true;
		logger.GLogTrap("PoseFail;a;"+cString::FromFloat64(static_cast<float>(a))+
						";last_a;"+cString::FromFloat64(static_cast<float>(myLast_a))+
						";act_theta;"+cString::FromFloat64(static_cast<float>(vActualPose.theta))+
						";b;"+cString::FromFloat64(static_cast<float>(b))+
						";x;"+cString::FromFloat64(static_cast<float>(x))+
						";qDesiredSteer;"+cString::FromFloat64(qDesiredSteering)+
						";LastDVel;"+cString::FromFloat64(myCruiseVelocity)+
						";qDesiredVel;"+cString::FromFloat64(qDesiredVelocity)+
						";reason;"+cString::FromInt(reason));
		WritePoseReached=true;

	}
	if (a<myLast_a)
	{
		myLast_a=a;
	}
	bool SameOrientation = fabs(vDesiredPose.theta-vActualPose.theta)<tSAME_ORIENTATION_DEG && fabs(y)<tSAME_ORIENTATION_Y;
	if ((a<PoseReached || SameOrientation) && !hasFinishedPoseProc)
	{
		hasFinishedPoseProc=true;
		reason=1;
		//Forward desired velocity and steering
		qDesiredVelocity=vDesiredPose.speed;
		logger.GLogTrap("PoseReached;a;"+cString::FromFloat64(static_cast<float>(a))+
						";last_a;"+cString::FromFloat64(static_cast<float>(myLast_a))+
						";act_theta;"+cString::FromFloat64(static_cast<float>(vActualPose.theta))+
						";b;"+cString::FromFloat64(static_cast<float>(b))+
						";x;"+cString::FromFloat64(static_cast<float>(x))+
						";qDesiredSteer;"+cString::FromFloat64(qDesiredSteering)+
						";LastDVel;"+cString::FromFloat64(myCruiseVelocity)+
						";qDesiredVel;"+cString::FromFloat64(qDesiredVelocity)+
						";reason;"+cString::FromInt(reason));
		WritePoseReached=true;
	}

	if (hasFinishedPoseProc)
	{
		qDesiredVelocity=vDesiredPose.speed;
	}
	else
	{
		//Destination pose not in window
		//Interpolate velocity according to DECELERATION
		qDesiredVelocity=myCruiseVelocity;
		if (vDesiredPose.speed!=0)
		{
			reason=3;
		}
		else
		{
			//Check for decceleration
			float mStopDist = (myCruiseVelocity*myCruiseVelocity)/(2.0*Decceleration);
			if (a<=mStopDist)
			{
				//Interpolate desired velocity
				qDesiredVelocity = (myCruiseVelocity*a)/mStopDist;
				reason=4;
			}
			else
			{
				qDesiredVelocity=myCruiseVelocity;
				reason=5;
			}
		}
	}

	if (vLog)
	{
		logger.GLogTrap("PBP;a;"+cString::FromFloat64(static_cast<float>(a))+
						";last_a;"+cString::FromFloat64(static_cast<float>(myLast_a))+
						";d_tht;"+cString::FromFloat64(myDeltaTheta)+","+cString::FromFloat64(limitedDeltaTheta)+
						";k_t;"+cString::FromFloat64(myGainTerm)+
						";act_tht;"+cString::FromFloat64(static_cast<float>(vActualPose.theta))+
						";b;"+cString::FromFloat64(static_cast<float>(b*cStdMath::MATH_RAD2DEG))+
						";AcD;"+cString::FromFloat64(static_cast<float>(x))+
						";AlD;"+cString::FromFloat64(static_cast<float>(y))+
						";qDSteer;"+cString::FromFloat64(qDesiredSteering)+
						";LastDVel;"+cString::FromFloat64(myCruiseVelocity)+
						";qDesVel;"+cString::FromFloat64(qDesiredVelocity)+
						";rsn;"+cString::FromInt(reason));
	}



	if (WritePoseReached)
	{
		WritePoseReachedBefore();
	}
	RETURN_NOERROR;

}

tResult SR_Controller::ProcessBlockVelocity(float vDesiredVelocity, float vMeasuredVelocity, bool vLog, float &qXMLSpeed)
{
	//Algorithm entry point
	//Inputs myMeasSpeed,mySpeedSetpoint
	//Previous values wL_mySpeedSetpoint,wL_myMeasSpeed
	//PID variables P_Gain,I_Gain,D_Gain
	//Outputs myRef_Component,myP_Component,myI_Component,myD_Component

	//Very simplistic approach to control: reference component is speed setpoint, P component is error multiplied with P gain
	float myRef_Component=vDesiredVelocity;
	float myError=vDesiredVelocity-vMeasuredVelocity;

	//Put error into I buffer
	mErrors.push_back(myError);

	int myErrorsSize=mErrors.size();
	//LOG_INFO("Size of I buffer ["+cString::FromInt(myErrorsSize)+"]");
	if (myErrorsSize>=myI_BufferSize)
	{
		mErrors.pop_front();

	}
	myErrorsSize=mErrors.size();
	float myI_Component=0;
	for (int i=0;i<myErrorsSize;i++)
	{
		myI_Component+=mErrors[i];
	}

	float myP_Component=myError*P_Gain;
	myI_Component/=myErrorsSize;
	myI_Component*=I_Gain;


	float myOutputSpeed=myRef_Component+myP_Component+myI_Component;

	//Standstill detection and deactivating of control
	if (vDesiredVelocity==0.0 && fabs(vMeasuredVelocity)<myStandstillWindow)
	{
		myOutputSpeed = 0.0;
	}
	if (myOutputSpeed>myMaximumCmd) {myOutputSpeed=myMaximumCmd;}
	if (myOutputSpeed<myMinimumCmd) {myOutputSpeed=myMinimumCmd;}


	qXMLSpeed=getLinearInterpolatedValue(myOutputSpeed);

	if (vLog)
	{
		cString myDataMessage=  "V_o;"+cString::FromFloat64(myOutputSpeed)+
				";V_ref;"+cString::FromFloat64(vDesiredVelocity)+
				";V_m;"+cString::FromFloat64(vMeasuredVelocity)+
				";E;"+cString::FromFloat64(myError)+
				";P;"+cString::FromFloat64(myP_Component)+
				";I;"+cString::FromFloat64(myI_Component)+
				";ESTOP;"+cString::FromBool(myEstop);//+
		//";SYS_ACTIVE;"+cString::FromBool(mySystemActive)+
		//";SPD_ZERO;"+cString::FromBool(mySpeedZero)+
		//";SPD_NEG;"+cString::FromBool(mySpeedNeg);
		logger.GLogTrap(myDataMessage);
	}
	RETURN_NOERROR;
}

tResult SR_Controller::ProcessBlockSteering(float vDesiredAngle, bool vLog, float &qXMLAngle)
{
	/*while (vDesiredAngle>=270)
	{
		vDesiredAngle-=360;
	}
	while (vDesiredAngle<-270)
	{
		vDesiredAngle+=360;
	}*/
	float mySteer=90.0-vDesiredAngle;
	if (mySteer<60.0)
	{
		mySteer=60.0;
	}
	if (mySteer>120.0)
	{
		mySteer=120.0;
	}
	qXMLAngle=mySteer;
	RETURN_NOERROR;
}


tResult SR_Controller::ProcessLights()
{
	bool mySpeedZero=(mySpeedSetpoint==0);
	bool mySpeedNeg=(actualPose.speed<0.3);



	bool wRE_SystemActive,wFE_SystemActive;
	bool wRE_BrakeLights,wFE_BrakeLights;
	bool wRE_RevLights,wFE_RevLights;
	fnc_Edge(mySystemActive,wL_mySystemActive,wRE_SystemActive,wFE_SystemActive);
	fnc_Edge(mySpeedZero && mySystemActive ,wL_myBrakeLights,wRE_BrakeLights,wFE_BrakeLights);
	fnc_Edge(mySpeedNeg && mySystemActive ,wL_myRevLights,wRE_RevLights,wFE_RevLights);


	if (wRE_SystemActive)
	{
		//LOG_INFO("RE_SYS_Active");
		myHeadLightRequired=true;
		myHeadLightVal=true;
	}
	else if(wFE_SystemActive)
	{
		//LOG_INFO("FE_SYS_Active");
		myHeadLightRequired=true;
		myHeadLightVal=false;
	}
	if (wRE_BrakeLights)
	{
		//LOG_INFO("RE_BRAKE");
		myBrakeLightRequired=true;
		myBrakeLightVal=true;
	}
	else if(wFE_BrakeLights)
	{
		//LOG_INFO("FE_BRAKE");
		myBrakeLightRequired=true;
		myBrakeLightVal=false;
	}
	if (wRE_RevLights)
	{
		//LOG_INFO("RE_REV");
		myRevLightRequired=true;
		myRevLightVal=true;
	}
	else if(wFE_RevLights)
	{
		//LOG_INFO("FE_REV");
		myRevLightRequired=true;
		myRevLightVal=false;
	}
	if (myHeadLightRequired)
	{
		myHeadLightRequired=false;
		SEND_WRAPPED_SAMPLE(outPinHeadLights,myHeadLightVal,_clock->GetTime());
	}
	else if (myRevLightRequired)
	{
		myRevLightRequired=false;
		SEND_WRAPPED_SAMPLE(outPinRevLights,myRevLightVal,_clock->GetTime());
	}
	else if (myBrakeLightRequired)
	{
		myBrakeLightRequired=false;
		SEND_WRAPPED_SAMPLE(outPinBrakeLights,myBrakeLightVal,_clock->GetTime());
	}
	RETURN_NOERROR;
}

tResult SR_Controller::WritePoseReachedBefore()
{
	SEND_WRAPPED_SAMPLE(outPinReachingPose,true,_clock->GetTime());
	RETURN_NOERROR;
	RETURN_NOERROR;
}

double SR_Controller::GetDistanceBetweenOdos(OdometryPose a, OdometryPose b)
{
	return sqrt(pow((a.x-b.x),2)+pow((a.y-b.y),2));
}

double SR_Controller::GetCrossDistanceFromReference(OdometryPose reference)
{
	float reference_rad=cStdMath::MATH_DEG2RAD*reference.theta;
	return (actualPose.x-reference.x)*sin(reference_rad)+(actualPose.y-reference.y)*cos(reference_rad);
}

float SR_Controller::ProcessUS(cString vName,WrappedSVInputPin &vPin,IMediaSample *vMediaSample, size_t &vCounter, std::vector<float> &vFilter,bool vLog)
{
	tFloat32 temp = 0;
	vPin.getValue(vMediaSample, &temp);
	if (vCounter < US_FILTER_SIZE) {
		vFilter.push_back(temp);
	} else {
		vFilter[vCounter % US_FILTER_SIZE] = temp;
	}
	float sum = 0;
	for (size_t i = 0; i < vFilter.size(); ++i) {
		sum += vFilter[i];
	}
	vCounter++;
	float tempDist=sum / static_cast<float>(vFilter.size());
	if  (vLog) {
		LOG_INFO(vName+cString::FromFloat64(tempDist)+"]");
	}
	return tempDist;
}

void SR_Controller::fnc_Edge(bool vData, bool &vLast, bool &RisingEdge, bool &FallingEdge)
{
	if (vData && !vLast)
	{
		RisingEdge=true;
		FallingEdge=false;
	}
	else if (!vData && vLast)
	{
		RisingEdge=false;
		FallingEdge=true;
	}
	else
	{
		RisingEdge=false;
		FallingEdge=false;
	}
	vLast=vData;
}

cString SR_Controller::GetStringPose(cString vName,OdometryPose vPose)
{
	return vName+";x;"+cString::FromFloat64(vPose.x)+";y;"+cString::FromFloat64(vPose.y)+";theta;"+cString::FromFloat64(vPose.theta);
}




