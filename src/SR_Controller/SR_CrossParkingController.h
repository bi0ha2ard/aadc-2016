#ifndef SR_CROSSPARKINGCONTROLLER_H
#define SR_CROSSPARKINGCONTROLLER_H

#include "SR_AbstractParkingController.h"

class SR_CrossParkingController : public AbstractParkingController
{
public:
	enum CrossParkingState
	{
		Ready,
		BeginProcedure,
		WaitStandstill,
		Left15,
		RevRight60,//Go reverse
		Left90,
		ReverseFinal0,
		ReverseFinal1,
		RightRevForLeftCorrect,
		LeftRevForLeftCorrect,
		LeftRevForRightCorrect,
		RightRevForRightCorrect,
		ReverseFinal2,
		WaitFinish,
		Finish,
	};
	SR_CrossParkingController(SR_Controller *vParent);
	virtual ~SR_CrossParkingController();
	void process(float &qDesiredVelocity, float &qDesiredSteering);
	void resetState(const CrossParkingState &value);

private:
	CrossParkingState state;
	void changeState(const CrossParkingState &value);

	// AbstractParkingController interface
protected:
	void setStateReady();
};



#endif // SR_PARKINGCONTROLLER_H
