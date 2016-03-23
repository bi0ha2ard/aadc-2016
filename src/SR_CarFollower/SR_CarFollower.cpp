#include "SR_CarFollower.h"
#include <map_data.h>
// Create filter shell

#undef LOG_INFO
#define LOG_INFO(str) logger.GLogTrap((str));
#define VEL_FILTER_SIZE 10

#define PROP_CYCLETIME "Timing::Timer control cycle [msec]"
#define PROP_SPACING "Spacing::Maximum distance to car [m]"
#define PROP_SPEEDSPACING "Spacing::Speed for compensation of distance [m/s]"

ADTF_FILTER_PLUGIN("SpaceRacer Sign Mapping", OID_SR_CARFOLLOWER, CarFollower)

CarFollower::CarFollower(const tChar *info) : cFilter(info),m_hTimer(NULL),logger(OID_SR_CARFOLLOWER)
{
	m_firstTime = tTrue;


	SetPropertyInt(PROP_CYCLETIME, 500);
	SetPropertyBool(PROP_CYCLETIME NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_CYCLETIME NSSUBPROP_DESCRIPTION, "The control step for the local sequence timer.");
	SetPropertyFloat(PROP_SPACING, 0.5);
	SetPropertyBool(PROP_SPACING NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_SPACING NSSUBPROP_DESCRIPTION, "The spacing maximum between cars");
	SetPropertyFloat(PROP_SPEEDSPACING, 1.0);
	SetPropertyBool(PROP_SPEEDSPACING NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_SPEEDSPACING NSSUBPROP_DESCRIPTION, "The speed to compensate distance between cars");

}

CarFollower::~CarFollower()
{

}

tResult CarFollower::processSigns(SignStruct &data)
{

	if (data.id == RoadSignTargetCar) {
		signTimer=8;
		//logger.log()<<"CarFollower::processSigns;"<<
		//			  data.pos.x<<";"<<
		//			  data.pos.y<<";"<<
		//			  "ID;"<<data.id<<endl;
		targetPose.x=data.pos.x;
		targetPose.y=data.pos.y;
		targetPose.speed=0;
		targetPose.theta=data.theta;
		logger.log()<<"TargetPose;"<<targetPose<<endl;
		//average the delta velocity of target pose
		tTimeStamp currTimeStamp=_clock->GetTime();
		tFloat64 timeDiff=(currTimeStamp-lastTimeStamp);
		lastTimeStamp=currTimeStamp;

		//Calculate relative velocity
		float relativeDistance=sqrt(pow(targetPose.x-currentPose.x,2)+pow(targetPose.y-currentPose.y,2));
		float deltaRelativeDistance=relativeDistance-lastRelativeDistance;
		lastRelativeDistance=relativeDistance;
		float relativeVelocity;
		if (timeDiff>0) {
			relativeVelocity=(deltaRelativeDistance*1000000)/timeDiff;
		} else {
			relativeVelocity=0;
		}

		//Calculate relative velocity
		float absoluteDistance=sqrt(pow(targetPose.x,2)+pow(targetPose.y,2));
		float deltaAbsoluteDistance=absoluteDistance-lastAbsoluteDistance;
		lastAbsoluteDistance=absoluteDistance;
		float absoluteVelocity;
		if (timeDiff>0) {
			absoluteVelocity=(deltaAbsoluteDistance*1000000)/timeDiff;
		} else {
			absoluteVelocity=0;
		}

		/*if (timeDiff>250*1000)
		{
			//Target reacquired after long time (250 ms), do not calculate velocity
			commandVelocity=0;
			RETURN_NOERROR;
		}*/

		//Filter relative velocity
		if (filterRelativeTargetVelocityCounter < VEL_FILTER_SIZE) {
			filterRelativeTargetVelocity.push_back(relativeVelocity);
		}
		else
		{
			filterRelativeTargetVelocity[filterRelativeTargetVelocityCounter % VEL_FILTER_SIZE]=relativeVelocity;
		}
		float sum = 0;
		for (size_t i = 0; i < filterRelativeTargetVelocity.size(); ++i) {
			sum += filterRelativeTargetVelocity[i];
		}
		filterRelativeTargetVelocityCounter++;
		targetRelativeVelocityFiltered=sum/filterRelativeTargetVelocity.size();


		//Filter absolute velocity
		if (filterAbsoluteTargetVelocityCounter < VEL_FILTER_SIZE) {
			filterAbsoluteTargetVelocity.push_back(absoluteVelocity);
		}
		else
		{
			filterAbsoluteTargetVelocity[filterAbsoluteTargetVelocityCounter % VEL_FILTER_SIZE]=absoluteVelocity;
		}
		sum = 0;
		for (size_t i = 0; i < filterAbsoluteTargetVelocity.size(); ++i) {
			sum += filterAbsoluteTargetVelocity[i];
		}
		filterAbsoluteTargetVelocityCounter++;
		targetAbsoluteVelocityFiltered=sum/filterAbsoluteTargetVelocity.size();

		float D = GetPropertyFloat(PROP_SPACING);
		float a = GetPropertyFloat(PROP_SPEEDSPACING);
		float compensatePos = a*(relativeDistance-D)/D;
		if (compensatePos<0)
		{
			compensatePos = 0;
		}

		if (relativeDistance<D)
		{
			commandVelocity=0.8*targetAbsoluteVelocityFiltered;
		}
		else
		{
			commandVelocity=currentPose.speed+0.8*compensatePos;
		}

		if (commandVelocity>1.5)
		{
			logger.log()<<"CommandVelocityExceedMax;"<<commandVelocity<<endl;
			commandVelocity=1.5;
		}

		logger.log()<<"RelVelocity;"<<targetRelativeVelocityFiltered<<";AbsVelocity;"<<currentPose.speed<<endl;//<<";Time;"<<timeDiff<<";FilteredVelocity;"<<targetRelativeVelocityFiltered<<endl;
		//logger.log()<<"AbsVelocity;Calc;"<<absVelocity<<";Distance;"<<absDistance<<";Time;"<<timeDiff<<";FilteredVelocity;"<<targetAbsoluteVelocityFiltered<<endl;
		//logger.log()<<"CommandVelocity;"<<commandVelocity<<endl;
	}

	RETURN_NOERROR;
}

void CarFollower::enableFollowing()
{
	followingActive=true;
}

void CarFollower::disableFollowing()
{
	followingActive=false;
}


tResult CarFollower::Init(adtf::cFilter::tInitStage stage, ucom::IException **__exception_ptr)
{
	// call base implementation
	RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))

			if (stage == StageFirst) {

		INIT_WRAPPED_PIN(keyboardCommandPin, "KeyboardCommand");
		REGISTER_MEDIA_PIN(posePin, MEDIA_TYPE_ODOMETRY, MEDIA_SUBTYPE_ODOMETRY_POSE, "Current_Pose");
		REGISTER_MEDIA_PIN(signInputPin, MEDIA_TYPE_MAPDATA, MEDIA_SUBTYPE_MAPDATA_SIGN, "Sign_output");
		INIT_WRAPPED_PIN(controllerPoseReachedPin, "PoseReached");

		REGISTER_MEDIA_PIN(controllerOutputPin, MEDIA_TYPE_CONTROLLERCOMMAND, MEDIA_SUBTYPE_CONTROLLER_COMMAND, "Command");


	} else if (stage == StageNormal) {
		currentPose.x=0;
		currentPose.y=0;
		currentPose.theta=0;
		currentPose.speed=0;
		lastRelativeDistance=0;
		debug1=true;
		followingActive=false;
		filterRelativeTargetVelocity.clear();
		filterRelativeTargetVelocityCounter=0;
		filterAbsoluteTargetVelocity.clear();
		filterAbsoluteTargetVelocityCounter=0;
		commandVelocity=0.0;
	} else if (stage == StageGraphReady) {

	}
	RETURN_NOERROR;
}

tResult CarFollower::Shutdown(adtf::cFilter::tInitStage stage, ucom::IException **__exception_ptr)
{
	if (stage == StageGraphReady) {

	} else if (stage == StageNormal) {

	} else if (stage == StageFirst) {

	}

	// call base implementation
	return cFilter::Shutdown(stage, __exception_ptr);
}

tResult CarFollower::OnPinEvent(adtf::IPin *source, tInt eventCode, tInt param1, tInt param2, adtf::IMediaSample *mediaSample)
{
	if (eventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		if (mediaSample != NULL)
		{
			if (source == &posePin) {
				READ_MEDIA_PIN(mediaSample, OdometryPose, currentPose);
			}
			else if (source == keyboardCommandPin.ppin) {
				float val;
				keyboardCommandPin.getValue(mediaSample, &val);
				handleKeyboardCommand(val);
			}
			else if (source == &signInputPin) {
				SignStruct s;
				READ_MEDIA_PIN(mediaSample, SignStruct, s);
				processSigns(s);
			}


		}
	}

	RETURN_NOERROR;
}

void CarFollower::ProcessInput()
{

}

tResult CarFollower::Run(tInt nActivationType, const tVoid *pvUserData, tInt nUserDataSize, IException **__exception_ptr)
{
	ControllerCommand command;
	command.type=DesiredPose;
	signTimer--;
	if (signTimer<0)
	{
		signTimer=-1;
		if (followingActive)
		{
			followingActive=false;
			commandVelocity=0;
			logger.log()<<"NoSign;"<<endl;
		}
	}

	bool doSendCommand=false;
	if (followingActive)
	{
		command.pose=targetPose;
		command.cruiseVelocity=commandVelocity;
		doSendCommand=true;
		logger.log()<<"SendCommand;"<<command<<endl;
		//logger.log()<<"Relative;"<<targetRelativeVelocityFiltered<<";Absolute;"<<targetAbsoluteVelocityFiltered<<";Distance;"<<lastRelativeDistance<<endl;
	}
	else
	{
		currentPose.speed=0;
		command.pose=currentPose;
		command.cruiseVelocity=.5;
		if (lastFollowingActive)
		{
			doSendCommand=true;
		}
	}
	lastFollowingActive=followingActive;
	if (doSendCommand && command.cruiseVelocity>0)
	{
		sendDesiredPose(command);
	}
	RETURN_NOERROR;

}

tResult CarFollower::Start(IException **__exception_ptr)
{
	createTimer();
	RETURN_NOERROR;
}

tResult CarFollower::Stop(IException **__exception_ptr)
{
	__synchronized_obj(m_oCriticalSectionTimerSetup);
	destroyTimer(__exception_ptr);
	return cFilter::Stop(__exception_ptr);
}


void CarFollower::handleKeyboardCommand(float command)
{
	if (command == 5.0f) {
		enableFollowing();
	} else if (command == 6.0f) {
		disableFollowing();
	}
}

tResult CarFollower::createTimer()
{

	// creates timer with 0.5 sec
	__synchronized_obj(m_oCriticalSectionTimerSetup);
	// additional check necessary because input jury structs can be mixed up because every signal is sent three times
	if (m_hTimer == NULL)
	{
		adtf_util::cString myTimerName(OIGetInstanceName());
		myTimerName.Append(".timer");
		if (debug1) LOG_INFO("Generating timer with string: ["+myTimerName+"] and cycle time ["+cString::FromInt(GetPropertyInt(PROP_CYCLETIME))+"]");
		//LOG_INFO("SR_Controller: ON");
		m_hTimer = _kernel->TimerCreate(tTimeStamp(GetPropertyInt(PROP_CYCLETIME)*1000), 0, static_cast<IRunnable*>(this),
										NULL, NULL, 0, 0, myTimerName);
	}

	RETURN_NOERROR;
}

tResult CarFollower::destroyTimer(__exception)
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

tResult CarFollower::sendDesiredPose(const ControllerCommand &p)
{
	SEND_MEDIA_SAMPLE(controllerOutputPin, ControllerCommand, p, _clock->GetStreamTime());
	RETURN_NOERROR;
}
