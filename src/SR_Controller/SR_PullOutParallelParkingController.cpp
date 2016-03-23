#include "SR_PullOutParallelParkingController.h"
#include "SR_Controller.h"

#undef LOG_INFO
#define LOG_INFO(str) parent->logger.GLogTrap((str));

SR_PullOutParallelParkingController::SR_PullOutParallelParkingController(SR_Controller *vParent) : AbstractParkingController (vParent, "PullOutParallelParkingController")
{
}

SR_PullOutParallelParkingController::~SR_PullOutParallelParkingController()
{

}

void SR_PullOutParallelParkingController::process(float &qDesiredVelocity, float &qDesiredSteering)
{
	qDesiredVelocity=parent->myIdleSpeed;
	qDesiredSteering=0;
	float myCruiseVelocity=0.23;
	accumulatedTheta=fabs(referencePose.theta-parent->actualPose.theta);
	float OrientationDiff,OrientationSum;
	OrientationDiff=referencePose.theta-parent->actualPose.theta;
	OrientationSum=referencePose.theta+parent->actualPose.theta;
	if (OrientationSum>360)
	{
		OrientationSum-=360;
	}
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
	bool somethingInFront=parent->CheckUSFrontAll(0.18);
	bool somethingInRear=parent->CheckUSRear(0.2);
	float crossDist=parent->GetDistanceBetweenOdos(referencePose,parent->actualPose)*
			sin(referencePose.theta*cStdMath::MATH_DEG2RAD-atan2(referencePose.y-parent->actualPose.y,referencePose.x-parent->actualPose.x));
	//float crossDist=parent->GetDistanceBetweenOdos(referencePose,parent->actualPose)*sin(OrientationSum*cStdMath::MATH_DEG2RAD);
	//if (doLog) {parent->logger.log()<<"Cross track distance ["<<crossDist<<"]"<<endl;}
	//LOG_INFO("Theta difference ["+cString::FromFloat64(accumulatedTheta)+"] OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");//LOG_INFO("Something in rear ["+cString::FromBool(somethingInRear)+"]");
	if (cyclesWait)
	{
		cyclesWait--;
	}
	switch (state)
	{
	case Ready:
		break;
	case BeginProcedure:
		//If car corner is found (side right sensor detects something), go slowly in reverse until corner edge is triggered
		//If car corner is not found (side right sensor doesn't detect something), go slowly in forward until corner edge is triggered
		if (doLog) {LOG_INFO("BeginProcedure");}
		//if (doLog) {LOG_INFO("US RIGHT ["+cString::FromFloat64(parent->myUS.Dist_SideRight)+"]");}
		if (parent->myUS.Dist_Front>0.2)
		{
			changeState(ChangeOrientationLeftFwd);
			cyclesWait=3;
		}
		else
		{
			changeState(ChangeOrientationRightRev);
			cyclesWait=3;
		}
		break;
	case ChangeOrientationLeftFwd:
		//LOG_INFO("FindCarCornerFwd");
		qDesiredVelocity=myCruiseVelocity;
		qDesiredSteering=STEER_MAX_LEFT;
		if (somethingInFront && cyclesWait<=0)
		{
			changeState(ChangeOrientationRightRev);
			cyclesWait=7;
		}
		else if (fabs(OrientationDiff)>40)
		{
			changeState(ForwardUntilOut);
		}
		break;
	case ChangeOrientationRightRev:
		//LOG_INFO("FindCarCornerFwd");
		qDesiredVelocity=-myCruiseVelocity;
		qDesiredSteering=STEER_MAX_RIGHT;
		if (somethingInRear && cyclesWait<=0)
		{
			changeState(ChangeOrientationLeftFwd);
			cyclesWait=7;
		}
		else if (fabs(OrientationDiff)>40)
		{
			changeState(ForwardUntilOut);
		}
		break;
	case ForwardUntilOut:
		//LOG_INFO("FindCarCornerFwd");
		qDesiredVelocity=myCruiseVelocity;
		qDesiredSteering=STEER_STRAIGHT;
		//if (parent->GetDistanceBetweenOdos(referencePose,parent->actualPose)>0.4)
		if (crossDist>0.25)
		{
			changeState(RightUntilStraight);
		}
		break;
	case RightUntilStraight:
		//LOG_INFO("FindCarCornerFwd");
		qDesiredVelocity=myCruiseVelocity;
		qDesiredSteering=STEER_MAX_RIGHT;
		if (fabs(OrientationDiff)<5)
		{
			LOG_INFO("Finished pull out parking.");
			parent->WritePoseReachedBefore();
			resetState(Ready);
			break;
		}

	}
	//LOG_INFO("qDesiredVelocity:["+cString::FromFloat64(qDesiredVelocity)+"] OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");
}

void SR_PullOutParallelParkingController::resetState(const PullOutParallelParkingState &value)
{
	if (doLog) {parent->logger.log()<<name<<";resetState;From;"<<state<<";To;"<<value<<endl;}
	state = value;
	stateOk = false;
}

void SR_PullOutParallelParkingController::changeState(const SR_PullOutParallelParkingController::PullOutParallelParkingState &value)
{
	if (doLog) {parent->logger.log()<<name<<";changeState;From;"<<state<<";To;"<<value<<endl;}
	state = value;
}

void SR_PullOutParallelParkingController::setStateReady()
{
	state = BeginProcedure;
}

