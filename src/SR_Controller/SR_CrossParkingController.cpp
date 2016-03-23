#include "SR_CrossParkingController.h"
#include "SR_Controller.h"

#undef LOG_INFO
#define LOG_INFO(str) parent->logger.GLogTrap((str));

SR_CrossParkingController::SR_CrossParkingController(SR_Controller *vParent) : AbstractParkingController (vParent, "CrossParkingController")
{
}

SR_CrossParkingController::~SR_CrossParkingController()
{

}

void SR_CrossParkingController::process(float &qDesiredVelocity, float &qDesiredSteering)
{
	qDesiredVelocity=parent->myIdleSpeed;
	qDesiredSteering=0;
	float myCruiseVelocity=0.35;
	accumulatedTheta=fabs(referencePose.theta-parent->actualPose.theta);
	float OrientationDiff;
	OrientationDiff=referencePose.theta-parent->actualPose.theta;
	if (accumulatedTheta>180)
	{
		accumulatedTheta-=360;
		if (referencePose.theta>parent->actualPose.theta)
		{
			OrientationDiff-=360;
		}
		else
		{
			OrientationDiff+=360;
		}
	}
	//float crossDist=parent->GetDistanceBetweenOdos(referencePose,parent->actualPose)*
	//		sin(referencePose.theta*cStdMath::MATH_DEG2RAD-atan2(referencePose.y-parent->actualPose.y,referencePose.x-parent->actualPose.x));
	float crossDist=parent->GetCrossDistanceFromReference(referencePose);
	float MagOrientationFix=3;
	if (cyclesWait)
	{
		cyclesWait--;
	}
	//LOG_INFO("Theta difference ["+cString::FromFloat64(accumulatedTheta)+"] OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");//LOG_INFO("Something in rear ["+cString::FromBool(somethingInRear)+"]");
	if (doLog) {parent->logger.log()<<"CrossTrack;"<<crossDist<<";"<<
									  "ActDist;"<<parent->GetDistanceBetweenOdos(referencePose,parent->actualPose)<<";"<<
									  "ActTheta;"<<parent->actualPose.theta<<";"<<
									  endl;}
	switch (state)
	{
	case Ready:
		break;
	case BeginProcedure:
		changeState(WaitStandstill);
		cyclesWait=5;
		break;
	case WaitStandstill:
		if (cyclesWait<=0)
		{
			changeState(Left15);
		}
		break;
	case Left15:
		qDesiredSteering=STEER_MAX_LEFT;
		qDesiredVelocity=myCruiseVelocity;
		if (fabs(OrientationDiff)>15)
		{
			if (doLog) {LOG_INFO("Switch to InitialReverseRight with OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");}
			changeState(RevRight60);
		}
		//LOG_INFO("InitialLeft");
		break;
	case RevRight60:
		qDesiredSteering=STEER_MAX_RIGHT;
		qDesiredVelocity=-myCruiseVelocity;
		if (fabs(OrientationDiff)>80 || parent->myUS.Dist_RearLeft<0.15)
		{
			if (doLog) {LOG_INFO("Switch to MatchCarOrientationReverse with OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");}
			changeState(Left90);
		}
		//LOG_INFO("InitialReverseRight");
		break;
	case Left90:
		qDesiredSteering=STEER_MAX_LEFT;
		qDesiredVelocity=myCruiseVelocity;
		if (fabs(OrientationDiff)>87)
		{
			if (doLog) {LOG_INFO("Switch to ReverseFinal0 with OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");}
			changeState(ReverseFinal0);
		}
		break;
	case ReverseFinal0:
		qDesiredSteering=STEER_STRAIGHT;
		qDesiredVelocity=-myCruiseVelocity;
		if (doLog) {parent->logger.log()<<name<<";ReverseFinal0;ActDist["<<parent->GetDistanceBetweenOdos(referencePose,parent->actualPose)<<"]"<<endl;}
		//if (parent->GetDistanceBetweenOdos(referencePose,parent->actualPose)>0.40)
		if (fabs(crossDist)>0.25)
		{
			if (doLog) {LOG_INFO("Switch to ReverseFinal1 with OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");}
			changeState(ReverseFinal1);
		}
		break;
	case ReverseFinal1:
		qDesiredSteering=STEER_STRAIGHT;
		qDesiredVelocity=-myCruiseVelocity;
		if (doLog) {
			parent->logger.log() << "ReverseFinal1. Side left ["<<parent->myUS.Dist_SideLeft<<"] Side right ["<<parent->myUS.Dist_SideRight<<"]"<<endl;
		}
		//Car is straight
		changeState(ReverseFinal2);
		/*
		if (carOnLeft && carOnRight)
		{
			if (parent->myUS.Dist_SideLeft-parent->myUS.Dist_SideRight>carOnMiddleThreshold)
			{
				if (doLog) {LOG_INFO("Right car too close. OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");}
				//Right car is too close
				changeState(LeftRevForRightCorrect);
			}
			else if (parent->myUS.Dist_SideRight-parent->myUS.Dist_SideLeft>carOnMiddleThreshold)
			{
				if (doLog) {LOG_INFO("Left car too close. OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");}
				//Left car is too close
				changeState(RightRevForLeftCorrect);
			}
			else
			{
				if (doLog) {LOG_INFO("Two cars: car is in middle. OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");}
				//Car is straight
				changeState(ReverseFinal2);
			}
		}
		else if (carOnLeft)
		{
			if (parent->myUS.Dist_SideLeft<singleCarThreshold)
			{
				if (doLog) {LOG_INFO("Single car on left, too close. OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");}
				//Left car is too close
				changeState(RightRevForLeftCorrect);
			}
			else
			{
				if (doLog) {LOG_INFO("Single car on left, dist is in ok. OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");}
				//Car is straight
				changeState(ReverseFinal2);
			}
		}
		else if (carOnRight)
		{
			if (parent->myUS.Dist_SideRight<singleCarThreshold)
			{
				if (doLog) {LOG_INFO("Single car on right, too close. OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");}
				//Left car is too close
				changeState(LeftRevForLeftCorrect);
			}
			else
			{
				if (doLog) {LOG_INFO("Single car on right, dist is in ok. OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");}
				//Car is straight
				changeState(ReverseFinal2);
			}
		}
		else
		{
			if (doLog) {LOG_INFO("No other cars, dist is ok. OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");}
			//Car is straight
			changeState(ReverseFinal2);
		}
		*/
		break;
	case RightRevForLeftCorrect:
		qDesiredSteering=STEER_MAX_RIGHT;
		qDesiredVelocity=-myCruiseVelocity;
		if (fabs(OrientationDiff)>90+MagOrientationFix)
		{
			changeState(LeftRevForLeftCorrect);
		}
		break;
	case LeftRevForLeftCorrect:
		qDesiredSteering=STEER_MAX_LEFT;
		qDesiredVelocity=-myCruiseVelocity;
		if (fabs(OrientationDiff)<92)
		{
			changeState(ReverseFinal2);
		}
		break;
	case LeftRevForRightCorrect:
		qDesiredSteering=STEER_MAX_LEFT;
		qDesiredVelocity=-myCruiseVelocity;
		if (fabs(OrientationDiff)<90-MagOrientationFix)
		{
			changeState(RightRevForRightCorrect);
		}
		break;
	case RightRevForRightCorrect:
		qDesiredSteering=STEER_MAX_RIGHT;
		qDesiredVelocity=-myCruiseVelocity;
		if (fabs(OrientationDiff)>88)
		{
			changeState(ReverseFinal2);
		}
		break;
	case ReverseFinal2:
		qDesiredSteering=STEER_STRAIGHT;
		qDesiredVelocity=-myCruiseVelocity;
		//if (doLog) {parent->logger.log()<<name<<";ReverseFinal2;ActDist["<<parent->GetDistanceBetweenOdos(referencePose,parent->actualPose)<<"]"<<endl;}
		//if (parent->GetDistanceBetweenOdos(referencePose,parent->actualPose)>0.65 || parent->myUS.Dist_Rear<0.3)
		if (fabs(crossDist)>0.75-parent->GetDeccDist() || parent->myUS.Dist_Rear<0.3)
		{
			changeState(WaitFinish);
			cyclesWait=40;
		}
		break;
	case WaitFinish:
		if (cyclesWait<=0)
		{
			changeState(Finish);
		}
		break;
	case Finish:
		if (cyclesWait<=0)
		{
			parent->logger.log()<<name<<parent->GetStringPose("Actual",parent->actualPose)<<endl;
			parent->logger.log()<<name<<";Finished"<<endl;
			parent->WritePoseReachedBefore();
			resetState(Ready);
		}
		break;
	}
}

void SR_CrossParkingController::resetState(const CrossParkingState &value)
{
	if (doLog) {parent->logger.log()<<name<<";resetState;From;"<<state<<";To;"<<value<<endl;}
	state = value;
	stateOk = false;
}

void SR_CrossParkingController::changeState(const SR_CrossParkingController::CrossParkingState &value)
{
	if (doLog) {parent->logger.log()<<name<<";changeState;From;"<<state<<";To;"<<value<<endl;}
	state = value;
}

void SR_CrossParkingController::setStateReady()
{
	state = BeginProcedure;
}

