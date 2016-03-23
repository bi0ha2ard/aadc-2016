#ifndef SR_PULLOUTPARALLELPARKINGCONTROLLER_H
#define SR_PULLOUTPARALLELPARKINGCONTROLLER_H

#include "SR_AbstractParkingController.h"

class SR_PullOutParallelParkingController : public AbstractParkingController
{
public:
	enum PullOutParallelParkingState
	{
		Ready,
		BeginProcedure,
		ChangeOrientationLeftFwd,
		ChangeOrientationRightRev,
		ForwardUntilOut,
		RightUntilStraight
	};
	SR_PullOutParallelParkingController(SR_Controller *vParent);
	virtual ~SR_PullOutParallelParkingController();
	void process(float &qDesiredVelocity, float &qDesiredSteering);
	void resetState(const PullOutParallelParkingState &value);

private:
	PullOutParallelParkingState state;
	void changeState(const PullOutParallelParkingState &value);

	// AbstractParkingController interface
protected:
	void setStateReady();
};



#endif // SR_PARKINGCONTROLLER_H
