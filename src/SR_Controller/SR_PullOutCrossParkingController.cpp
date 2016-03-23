#include "SR_PullOutCrossParkingController.h"
#include "SR_Controller.h"

#undef LOG_INFO
#define LOG_INFO(str) parent->logger.GLogTrap((str));

SR_PullOutCrossParkingController::SR_PullOutCrossParkingController(SR_Controller *vParent) : AbstractParkingController (vParent, "PullOutCrossParkingController")
{
}

SR_PullOutCrossParkingController::~SR_PullOutCrossParkingController()
{

}

void SR_PullOutCrossParkingController::process(float &qDesiredVelocity, float &qDesiredSteering)
{
	qDesiredVelocity=parent->myIdleSpeed;
	qDesiredSteering=0;
	float myApproachVelocity=0.3;
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
	if (cyclesWait)
	{
		cyclesWait--;
	}
	//LOG_INFO("Theta difference ["+cString::FromFloat64(accumulatedTheta)+"] OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");//LOG_INFO("Something in rear ["+cString::FromBool(somethingInRear)+"]");
	switch (state)
	{
	case Ready:
		break;
	case BeginProcedure:
		changeState(Left15);
		break;
	case Left15:
		//LOG_INFO("FindCarCornerFwd");
		qDesiredVelocity=myApproachVelocity;
		qDesiredSteering=STEER_STRAIGHT;
		//if (fabs(OrientationDiff)>15)
		if (parent->GetDistanceBetweenOdos(referencePose,parent->actualPose)>0.25)
		{
			changeState(WaitStandstill);
			cyclesWait=10;
		}
		break;
	case WaitStandstill:
		if (cyclesWait<=0)
		{
			changeState(Right45);
		}
		break;
	case Right45:
		qDesiredVelocity=myApproachVelocity;
		qDesiredSteering=STEER_MAX_RIGHT;
		if (fabs(OrientationDiff)>45)
		{
			changeState(LeftRev60);
		}
		break;
	case LeftRev60:
		qDesiredVelocity=-myApproachVelocity;
		qDesiredSteering=STEER_MAX_LEFT;
		if (fabs(OrientationDiff)>60)
		{
			changeState(Right90);
		}
		break;
	case Right90:
		qDesiredVelocity=myApproachVelocity;
		qDesiredSteering=STEER_MAX_RIGHT;
		if (fabs(OrientationDiff)>90)
		{
			changeState(Finish);
		}
		break;
	case Finish:
	{
		parent->logger.log()<<name<<";Finished"<<endl;
		parent->WritePoseReachedBefore();
		resetState(Ready);
		break;
	}


	}
	//LOG_INFO("qDesiredVelocity:["+cString::FromFloat64(qDesiredVelocity)+"] OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");
}

void SR_PullOutCrossParkingController::resetState(const PullOutCrossParkingState &value)
{
	if (doLog) {parent->logger.log()<<name<<";resetState;From;"<<state<<";To;"<<value<<endl;}
	state = value;
	stateOk = false;
}

void SR_PullOutCrossParkingController::changeState(const SR_PullOutCrossParkingController::PullOutCrossParkingState &value)
{
	if (doLog) {parent->logger.log()<<name<<";changeState;From;"<<state<<";To;"<<value<<endl;}
	state = value;
}

void SR_PullOutCrossParkingController::setStateReady()
{
	state = BeginProcedure;
}

