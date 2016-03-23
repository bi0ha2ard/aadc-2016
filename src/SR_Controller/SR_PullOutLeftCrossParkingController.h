#ifndef SR_PULLOUTLEFTCROSSPARKINGCONTROLLER_H
#define SR_PULLOUTLEFTCROSSPARKINGCONTROLLER_H

#include "SR_AbstractParkingController.h"

class SR_PullOutLeftCrossParkingController : public AbstractParkingController
{
public:
	enum PullOutCrossParkingState
	{
		Ready,
		BeginProcedure,
		StraightFirst,
		WaitStandstill,
		LeftFwd90,
		Finish
	};
	SR_PullOutLeftCrossParkingController(SR_Controller *vParent);
	virtual ~SR_PullOutLeftCrossParkingController();
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
