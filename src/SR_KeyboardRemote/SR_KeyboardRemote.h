#ifndef _SIGNAL_PARKING_MODULE_H_
#define _SIGNAL_PARKING_MODULE_H_

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <adtf_graphics.h>
#include <adtf_utils.h>
#include "../misc/PinWrapper.h"
#include "../misc/SR_Maneuvers_List.h"
#include "../misc/SR_Logger.h"

#define OID_SR_KEYBOARD_REMOTE "adtf.aadc.keyboardremote"
#define DIGIT_COMMANDS 11
#define LETTER_COMMANDS 5

class SR_KeyboardRemote : public adtf::cFilter
{
    ADTF_FILTER(OID_SR_KEYBOARD_REMOTE, "SpaceRacer Keyboard Remote", adtf::OBJCAT_Application)

public:
    SR_KeyboardRemote(const tChar *info);
    virtual ~SR_KeyboardRemote();

	/*! creates the timer for the cyclic transmits*/
	tResult createTimer();

   /*! destroys the timer for the cyclic transmits*/
	tResult destroyTimer(__exception = NULL);

	/*! handle for the timer */
	tHandle m_hTimer;

protected:
	tResult Init(tInitStage eStage, IException **__exception_ptr);
	tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);
	//tHandle CreateView();
	//tResult ReleaseView();

private:

    SR_Logger myLogger;

	WrappedBVOutputPin qDataAvailable;
	WrappedBVOutputPin qPinF;
	WrappedBVOutputPin qPinG;
	WrappedBVOutputPin qPinH;
	WrappedBVOutputPin qPinJ;
	WrappedBVOutputPin qPinK;

	WrappedSVOutputPin qSteeringAngle;
	WrappedSVOutputPin qSpeed;
	WrappedSVOutputPin outPinCommand;

	vector<bool> myKeyboardCommands;
	vector<bool> wL_myKeyboardCommands;

    //below are active commands read from file
    bool myForward,myBackward,myRight,myLeft,myBrake,myStartup;
    cString myLastCommandLine;
    int myWatchdog;
    bool myLastWatchdogTriggered;
	bool myDebug1;
	tInt64 myPreviousLocalTime;
	tInt64 mySequenceTime;
    tFloat64 myComputedSpeed;
    tFloat64 myComputedSteer;
	tResult WriteOutMotionAndData(float vSpeed,float vSteer, bool vDataReady);
	//properties

float pPROP_INCREMENTAL_STEERING;// "Remote::Steering::Incremental acceleration [m/s]"
float pPROP_INC_STEER_RETURN_IDLE;// "Remote::Steering::Incremental return to idle steer [servo angle]"
float pPROP_IDLE_STEER;// "Remote::Steering::Straight position of wheels [servo angle]"
//Speed properties
float pPROP_INCREMENTAL_ACCELERATION;// "Remote::Speed::Incremental acceleration [m/s]"
float pPROP_INC_SPEED_RETURN_IDLE;// "Remote::Speed::Incremental return to idle speed [m/s]"
float pPROP_MAX_SPEED_FWD;// "Remote::Speed::Maximum forward speed [m/s]"
float pPROP_MAX_SPEED_BACK;// "Remote::Speed::Maximum backward speed [m/s]"
float pPROP_IDLE_SPEED;// "Remote::Speed::Neutral speed command [m/s]"
float pPROP_WATCHDOG_TIME;// "Remote::Watchdog [s]"

	/*! the critical section of the timer setup */
	cCriticalSection m_oCriticalSectionTimerSetup;
	// IRunnable interface
public:
	tResult Run(tInt nActivationType, const tVoid *pvUserData, tInt nUserDataSize, IException **__exception_ptr);


	// cFilter interface
protected:
	tResult Stop(IException **__exception_ptr);


	// cFilter interface
protected:
	tResult Start(IException **__exception_ptr);

    void ResetCommands();
	void SetHighSpeedLimits();
	void SetMediumSpeedLimits();
	void SetLowSpeedLimits();

	// cConfiguration interface
public:
	tResult PropertyChanged(const tChar *strName);
};


#endif
