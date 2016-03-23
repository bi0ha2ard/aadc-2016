#include "SR_AbstractParkingController.h"
#include "SR_Controller.h"

const int AbstractParkingController::STEER_MAX_LEFT=30;
const int AbstractParkingController::STEER_MAX_RIGHT=-30;
const int AbstractParkingController::STEER_STRAIGHT=0;

AbstractParkingController::AbstractParkingController(SR_Controller *parent, const char *name) :
	parent(parent), name(name)
{

}

AbstractParkingController::~AbstractParkingController() {}

void AbstractParkingController::Init(OdometryPose pose, bool vLog)
{
	parent->logger.log() << name<<";ref.x;" << pose.x <<
									   "y;" << pose.y <<
								   "theta;" << pose.theta <<
								   ";act.x;"<< parent->actualPose.x <<
									   "y;"<< parent->actualPose.y <<
								   "theta;"<< parent->actualPose.theta<<endl;
	referencePose=pose;
	accumulatedTheta=0;
	stateOk=true;
	doLog=vLog;
	setStateReady();
}

bool AbstractParkingController::getStateOk() const
{
	return stateOk;
}

