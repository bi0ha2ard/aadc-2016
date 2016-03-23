#ifndef ABSTRACTCONTROLLER_H
#define ABSTRACTCONTROLLER_H

#include <odometry_data.h>


class SR_Controller;

class AbstractParkingController
{
public:
	AbstractParkingController(SR_Controller *parent, const char* name);
	virtual ~AbstractParkingController();
	void Init(OdometryPose pose, bool vLog);

	bool getStateOk() const;
	virtual void process(float &qDesiredVelocity, float &qDesiredSteering) = 0;

protected:
	virtual void setStateReady() = 0;
	SR_Controller *parent;
	const char* name;
	OdometryPose referencePose;
	float accumulatedTheta;
	int cyclesWait;
	bool stateOk;
	bool doLog;

	static const int STEER_MAX_LEFT;
	static const int STEER_STRAIGHT;
	static const int STEER_MAX_RIGHT;
};

#endif // ABSTRACTCONTROLLER_H
