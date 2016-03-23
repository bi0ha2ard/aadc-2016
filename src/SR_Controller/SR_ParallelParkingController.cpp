#include "SR_ParallelParkingController.h"
#include "SR_Controller.h"

#undef LOG_INFO
#define LOG_INFO(str) parent->logger.GLogTrap((str));

SR_ParallelParkingController::SR_ParallelParkingController(SR_Controller *vParent) : AbstractParkingController (vParent, "ParallelParkingController")
{
}

SR_ParallelParkingController::~SR_ParallelParkingController()
{

}

void SR_ParallelParkingController::process(float &qDesiredVelocity, float &qDesiredSteering)
{
	qDesiredVelocity=parent->myIdleSpeed;
	qDesiredSteering=0;
	float myApproachVelocity=0.25;
	float myCruiseVelocity=0.28;
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
	bool OrientationAligned=fabs(OrientationDiff)<5;
	bool closeToOrientationAligned=fabs(OrientationDiff)<15;
	bool somethingInFront=parent->CheckUSFrontAll(0.2);
	bool somethingInRear=parent->CheckUSRear(0.25);
	bool somethingInRearLeft=parent->myUS.Dist_RearLeft<0.25;
	bool nothingInFront=parent->CheckUSNothingInFront();
	bool nothingInRear=parent->CheckUSNothingInRear();
	float distFront=parent->myUS.Dist_Front;
	float distRear=parent->myUS.Dist_Rear;
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
		if (doLog) {LOG_INFO("US RIGHT ["+cString::FromFloat64(parent->myUS.Dist_SideRight)+"]");}
		/*
		if (parent->myUS.Dist_SideRight<0.3)
		{
			changeState(FindCarCornerRev;
		}
		else
		{
			changeState(FindCarCornerFwd;
		}*/
		changeState(WaitStandstill);
		cyclesWait=10;
		break;
	case FindCarCornerFwd:
		//LOG_INFO("FindCarCornerFwd");
		qDesiredVelocity=myApproachVelocity;
		qDesiredSteering=STEER_STRAIGHT;
		if (parent->myUS.Dist_SideRight<0.3 || parent->GetDistanceBetweenOdos(referencePose,parent->actualPose)>0.1)
		{
			changeState(FindCarCornerRev);
		}
		break;
	case FindCarCornerRev:
		//LOG_INFO("FindCarCornerRev");
		qDesiredVelocity=-myApproachVelocity;
		qDesiredSteering=STEER_STRAIGHT;
		if (parent->myUS.Dist_SideRight>0.3)
		{
			changeState(WaitStandstill);
			cyclesWait=10;
		}
		break;
	case WaitStandstill:
		if (cyclesWait<=0)
		{
			changeState(InitialLeft);
		}
		break;
	case InitialLeft:
		qDesiredSteering=STEER_MAX_LEFT;
		qDesiredVelocity=myCruiseVelocity;
		if (fabs(OrientationDiff)>15)
		{
			if (doLog) {LOG_INFO("Switch to InitialReverseRight with OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");}
			changeState(InitialReverseRight);
		}
		//LOG_INFO("InitialLeft");
		break;
	case InitialReverseRight:
		qDesiredSteering=STEER_MAX_RIGHT;
		qDesiredVelocity=-myCruiseVelocity;
		if (fabs(OrientationDiff)>45)
		{
			if (doLog) {LOG_INFO("Switch to MatchCarOrientationReverse with OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");}
			changeState(MatchCarOrientationReverse);
		}
		//LOG_INFO("InitialReverseRight");
		break;
	case MatchCarOrientationReverse:
		qDesiredSteering=STEER_MAX_LEFT;
		qDesiredVelocity=-myCruiseVelocity;
		if (OrientationAligned)
		{
			changeState(WaitStandstill2);
			cyclesWait=10;
		}
		else if ((somethingInRearLeft || (closeToOrientationAligned && somethingInRear)) && cyclesWait<=0)
		{
			changeState(MatchCarOrientationForward);
			cyclesWait=7;
		}
		else if (nothingInRear)
		{
			;//StateOk=false;
		}
		//LOG_INFO("MatchCarOrientationReverse");
		break;
	case MatchCarOrientationForward:
		qDesiredSteering=STEER_MAX_RIGHT;
		qDesiredVelocity=myCruiseVelocity;
		if (OrientationAligned)
		{
			changeState(WaitStandstill2);
			cyclesWait=10;
		}
		else if (somethingInFront && cyclesWait<=0)
		{
			changeState(MatchCarOrientationReverse);
			cyclesWait=7;
		}
		else if (nothingInFront)
		{
			;//StateOk=false;
		}
		//LOG_INFO("MatchCarOrientationForward");
		break;
	case WaitStandstill2:
		if (cyclesWait<=0)
		{
			changeState(MatchCarDistance);
		}
		break;
	case MatchCarDistance:
		//LOG_INFO("MatchCarDistance");
		qDesiredSteering=0;
		somethingInFront=parent->CheckUSFrontCenterOnly(0.7);
		somethingInRear=parent->CheckUSRearCenterOnly(0.7);
		if (doLog) {LOG_INFO("distance to front:["+cString::FromFloat64(parent->myUS.Dist_Front)+"] distance to back:["+cString::FromFloat64(parent->myUS.Dist_Rear)+"]");}
		if (somethingInFront && somethingInRear)
		{
			if (doLog) {LOG_INFO("Switch to MatchCarDistanceSomethingFrontAndRear");}
			changeState(MatchCarDistanceSomethingFrontAndRear);
		}
		else if (somethingInFront)
		{
			if (doLog) {LOG_INFO("Switch to MatchCarDistanceSomethingFront");}
			changeState(MatchCarDistanceSomethingFront);
		}
		else if (somethingInRear)
		{
			if (doLog) {LOG_INFO("Switch to MatchCarDistanceSomethingRear");}
			changeState(MatchCarDistanceSomethingRear);
		}
		else
		{
			parent->WritePoseReachedBefore();
			if (doLog) {LOG_INFO("Finished with nothing in front or rear!");}
			changeState(Ready);
		}


		break;
	case MatchCarDistanceSomethingFrontAndRear:
		//Use existing logic
		if (fabs(distFront-distRear)<0.08)
		{
			parent->WritePoseReachedBefore();
			if (doLog) {LOG_INFO("Finished with something in front and something in rear!");}
			changeState(PreWaitFinish);
		}
		else if (distFront>distRear)
		{
			qDesiredVelocity=myApproachVelocity;
		}
		else
		{
			qDesiredVelocity=-myApproachVelocity;
		}
		break;
	case MatchCarDistanceSomethingFront:
		if (doLog) {LOG_INFO("Front dist ["+cString::FromFloat64(parent->myUS.Dist_Front)+"] Decc dist ["+cString::FromFloat64(parent->GetDeccDist())+"]");}
		if (distFront<parent->GetDeccDist()+0.2)
		{
			parent->WritePoseReachedBefore();
			if (doLog) {LOG_INFO("Finished with only something in front!");}
			changeState(PreWaitFinish);
		}
		else
		{
			qDesiredVelocity=myApproachVelocity;
		}
		break;
	case  MatchCarDistanceSomethingRear:
		if (doLog) {LOG_INFO("Rear dist ["+cString::FromFloat64(distRear)+"] Decc dist ["+cString::FromFloat64(parent->GetDeccDist())+"]");}
		if (distRear<parent->GetDeccDist()+0.18)
		{

			if (doLog) {LOG_INFO("Finished with only something in rear!");}
			changeState(PreWaitFinish);
		}
		else
		{
			qDesiredVelocity=-myApproachVelocity;
		}
		break;
	case PreWaitFinish:
		cyclesWait=1;
		changeState(WaitFinish);
		break;
	case WaitFinish:
		if (cyclesWait<=0)
		{
			LOG_INFO("Finished parallel parking.");
			parent->WritePoseReachedBefore();
			resetState(Ready);
		}
		break;
	}
	//LOG_INFO("qDesiredVelocity:["+cString::FromFloat64(qDesiredVelocity)+"] OrientationDiff ["+cString::FromFloat64(OrientationDiff)+"]");
}

void SR_ParallelParkingController::resetState(const ParallelParkingState &value)
{
	if (doLog) {parent->logger.log()<<name<<";resetState;From;"<<state<<";To;"<<value<<endl;}
	state = value;
	stateOk = false;
}

void SR_ParallelParkingController::changeState(const SR_ParallelParkingController::ParallelParkingState &value)
{
	if (doLog) {parent->logger.log()<<name<<";changeState;From;"<<state<<";To;"<<value<<endl;}
	state = value;
}

void SR_ParallelParkingController::setStateReady()
{
	state = BeginProcedure;
}

