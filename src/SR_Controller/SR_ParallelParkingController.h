#ifndef SR_PARALLELPARKINGCONTROLLER_H
#define SR_PARALLELPARKINGCONTROLLER_H

#include "SR_AbstractParkingController.h"

class SR_ParallelParkingController : public AbstractParkingController
{
public:
	enum ParallelParkingState
	{
		Ready,
		BeginProcedure,
		FindCarCornerFwd,
		FindCarCornerRev,
		WaitStandstill,
		InitialLeft,
		InitialReverseRight,
		MatchCarOrientationForward,
		MatchCarOrientationReverse,
		WaitStandstill2,
		MatchCarDistance,
		MatchCarDistanceSomethingFrontAndRear,
		MatchCarDistanceSomethingFront,
		MatchCarDistanceSomethingRear,
		PreWaitFinish,
		WaitFinish,
	};
	SR_ParallelParkingController(SR_Controller *vParent);
	virtual ~SR_ParallelParkingController();
	void process(float &qDesiredVelocity, float &qDesiredSteering);
	void resetState(const ParallelParkingState &value);

private:
	ParallelParkingState state;
	void changeState(const ParallelParkingState &value);

	// AbstractParkingController interface
protected:
	void setStateReady();
};



#endif // SR_PARKINGCONTROLLER_H
