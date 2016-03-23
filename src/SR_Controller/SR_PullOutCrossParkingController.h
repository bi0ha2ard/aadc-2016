#ifndef SR_PULLOUTCROSSPARKINGCONTROLLER_H
#define SR_PULLOUTCROSSPARKINGCONTROLLER_H

#include "SR_AbstractParkingController.h"

class SR_PullOutCrossParkingController : public AbstractParkingController
{
public:
	enum PullOutCrossParkingState
	{
		Ready,
		BeginProcedure,
		Left15,
		WaitStandstill,
		Right45,
		LeftRev60,
		Right90,
		Finish
	};
	SR_PullOutCrossParkingController(SR_Controller *vParent);
	virtual ~SR_PullOutCrossParkingController();
	void process(float &qDesiredVelocity, float &qDesiredSteering);
	void resetState(const PullOutCrossParkingState &value);

private:
	PullOutCrossParkingState state;
	void changeState(const PullOutCrossParkingState &value);

	// AbstractParkingController interface
protected:
	void setStateReady();
};



#endif // SR_PARKINGCONTROLLER_H
