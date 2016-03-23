#ifndef _SR_CONTROLLER_H_
#define _SR_CONTROLLER_H_

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <adtf_graphics.h>
#include <adtf_utils.h>
#include "../misc/PinWrapper.h"
#include "../misc/SR_Logger.h"
#include <odometry_data.h>
#include "SR_ParallelParkingController.h"
#include "SR_PullOutParallelParkingController.h"
#include "SR_CrossParkingController.h"
#include "SR_PullOutCrossParkingController.h"
#include "SR_PullOutLeftCrossParkingController.h"


using namespace adtf;
#define OID_SR_CONTROLLER "adtf.aadc.controller"

class SR_Controller : public adtf::cFilter
{
	ADTF_FILTER(OID_SR_CONTROLLER, "SpaceRacer Controller", adtf::OBJCAT_Application)


	public:


		//INPUT PINS
		//BOOL : Remote Keyboard Data Available -> RemoteAv
		//----------------------------------------------------------------
		/*
			cInputPin inPinRemoteAv;
			cObjectPtr<IMediaTypeDescription> mediaDescRemoteAv;
			tBool bufIDGetRemoteAv;
			tBufferID bufIDRemoteAv;
			*/
		WrappedBVInputPin inPinRemoteAv;
	//FLOAT32 : Remote Keyboard SetSpeed
	//----------------------------------------------------------------
	WrappedSVInputPin inPinRemoteKBSetSpeed;
	//FLOAT32 : Remote Keyboard SetSteering
	//----------------------------------------------------------------
	WrappedSVInputPin inPinRemoteKBSetSteering;



	//OdometryPose : ActualPose
	//----------------------------------------------------------------
	cInputPin inPinActualPose;
	//OdometryPose : DesiredPose
	//----------------------------------------------------------------
	cInputPin inPinCommand;

	//Ultrasonics
	WrappedSVInputPin inPinUltrasonic_Front;
	WrappedSVInputPin inPinUltrasonic_Front_Center_Left;
	WrappedSVInputPin inPinUltrasonic_Front_Center_Right;
	WrappedSVInputPin inPinUltrasonic_Front_Left;
	WrappedSVInputPin inPinUltrasonic_Front_Right;
	WrappedSVInputPin inPinUltrasonic_Rear;
	WrappedSVInputPin inPinUltrasonic_Rear_Left;
	WrappedSVInputPin inPinUltrasonic_Rear_Right;
	WrappedSVInputPin inPinUltrasonic_Side_Left;
	WrappedSVInputPin inPinUltrasonic_Side_Right;

	//OUTPUT PINS
	//FLOAT32 : Actuator
	//----------------------------------------------------------------
	WrappedSVOutputPin outPinSpeed;
	WrappedSVOutputPin outPinSteer;

	WrappedBVOutputPin outPinReachingPose;

	//BOOL : Lights
	//----------------------------------------------------------------
	WrappedBVOutputPin outPinHeadLights;
	WrappedBVOutputPin outPinBrakeLights;
	WrappedBVOutputPin outPinRevLights;

	OdometryPose actualPose;
	ControllerCommand myCommand;
	ControllerCommand lastCommand;
	tResult ProcessPose(IMediaSample *pMediaSample,OdometryPose &vPose,cString vName,bool vLog);
	tResult ProcessCommand(IMediaSample *pMediaSample,ControllerCommand &vPose,cString vName,bool vLog);

	bool ControllerCheckData();
	tResult ProcessBlockPose(OdometryPose vActualPose,OdometryPose vDesiredPose,bool vLog,float &qDesiredVelocity, float &qDesiredSteering);//outputs desired velocity[m/s] and desired steering [degrees]
	tResult ProcessBlockVelocity(float vDesiredVelocity, float vMeasuredVelocity, bool vLog, float &qXMLSpeed);//inputs desired velocity[m/s] and measured velocity[m/s] , outputs velocity servo angle
	tResult ProcessBlockSteering(float vDesiredAngle, bool vLog,float &qXMLAngle);//inputs desired steering, outputs servo angle

	tResult ProcessLights();
	tResult WritePoseReachedBefore();\
	double GetDistanceBetweenOdos(OdometryPose a,OdometryPose b);
	double GetCrossDistanceFromReference(OdometryPose reference);
	float ProcessUS(cString vName, WrappedSVInputPin &vPin, IMediaSample* vMediaSample, size_t &vCounter, std::vector<float> &vFilter, bool vLog);
	void fnc_Edge(bool vData,bool &vLast, bool &RisingEdge, bool &FallingEdge);
	cString GetStringPose(cString vName,OdometryPose vPose);
	SR_Controller(const tChar *info);
	virtual ~SR_Controller();

	/*! creates the timer for the cyclic transmits*/
	tResult createTimer();

	/*! destroys the timer for the cyclic transmits*/
	tResult destroyTimer(__exception = NULL);

	/*! handle for the timer */
	tHandle m_hTimer;

	/*! load settings from file */
	tResult LoadSettings();

	SR_Logger logger;

	tFloat64 myIdleSpeed;
	tFloat64 myIdleSpeedXML;
	tFloat64 myIdleSteerXML;

	bool CheckUSFrontAll();
	bool CheckUSFrontAll(float value);
	bool CheckUSFrontCenterOnly(float value);
	bool CheckUSFrontCenterLeftRight(float value);
	bool CheckUSRear();
	bool CheckUSRear(float value);
	bool CheckUSRearCenterOnly(float value);
	bool CheckUSNothingInFront();
	bool CheckUSNothingInRear();
	float CheckEstop(float desiredVelocity);

	typedef struct {
	std::vector<float> Filter_Front;
	std::vector<float> Filter_Rear;
	std::vector<float> Filter_FrontLeft;
	std::vector<float> Filter_FrontRight;
	std::vector<float> Filter_FrontCenterLeft;
	std::vector<float> Filter_FrontCenterRight;
	std::vector<float> Filter_RearLeft;
	std::vector<float> Filter_RearRight;
	std::vector<float> Filter_SideLeft;
	std::vector<float> Filter_SideRight;

	float Dist_Front;
	float Dist_Rear;
	float Dist_FrontLeft;
	float Dist_RearLeft;
	float Dist_FrontRight;
	float Dist_RearRight;
	float Dist_FrontCenterLeft;
	float Dist_FrontCenterRight;
	float Dist_SideLeft;
	float Dist_SideRight;

	size_t Counter_Front;
	size_t Counter_Rear;
	size_t Counter_FrontLeft;
	size_t Counter_RearLeft;
	size_t Counter_FrontRight;
	size_t Counter_RearRight;
	size_t Counter_FrontCenterLeft;
	size_t Counter_FrontCenterRight;
	size_t Counter_SideLeft;
	size_t Counter_SideRight;

	} UltrasonicData;
	UltrasonicData myUS;
	float GetDeccDist();

protected:
	tResult Init(tInitStage eStage, IException **__exception_ptr);
	tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);
	//tHandle CreateView();
	//tResult ReleaseView();

private:

	double fixAngle360Deg(double a);

	/*! the critical section for the on pin events */
	cCriticalSection m_critSecOnPinEvent;

	std::vector<float> IntErrorSteeringVector;
	size_t IntErrorSteeringCounter;
	float IntErrorSteering;

	bool myFinishedArduinoInit;
	int myWinkSteringArduinoReady;
	bool resetIBuffer;
	bool myLoadedSettings;
	bool myRemoteAvailable;
	bool mySystemActive;
	bool wL_mySystemActive;
	bool myResetActualPoseTimer;
	bool myResetDesiredPoseTimer;
	bool hasReachedPoseBefore;
	bool hasFinishedPoseProc;
	bool mySetLast_a;

	/*
	enum CrossParkingState
	{
		//Ready,
		cps_BeginProcedure
	} ;

	enum PullOutParallelState
	{
		//Ready,
		pops_BeginProcedure
	} ;

	enum PullOutCrossState
	{
		//Ready,
		pocs_BeginProcedure
	} ;*/

	SR_ParallelParkingController parallelParkingController;
	SR_PullOutParallelParkingController pullOutParallelParkingController;
	SR_CrossParkingController crossParkingController;
	SR_PullOutCrossParkingController pullOutCrossParkingController;
	SR_PullOutLeftCrossParkingController pullOutLeftCrossParkingController;

	float myCruiseVelocity;
	float myLast_a;
	float tPOSE_FAIL_HYSTERESIS;
	float tPOSE_FAIL_PARKING_HYSTERESIS;
	float tSAME_ORIENTATION_DEG;
	float tSAME_ORIENTATION_Y;
	//Controller settings
	tFloat32 P_Gain;
	tFloat32 I_Gain;
	tFloat32 P_Gain_S;
	tFloat32 I_Gain_S;
	int I_BufferSSize;
	tFloat32 DefaultCruiseVelocity;
	tFloat32 myMinimumCmd;
	tFloat32 myMaximumCmd;
	tFloat32 myStandstillWindow;
	tFloat32 PoseReached;
	tFloat32 Decceleration;

	typedef struct {
		tFloat32 pSTOP_US_DIST_FRONT;
		tFloat32 pSTOP_US_DIST_REAR;
		tFloat32 pMIN_VEL_US;
		tFloat32 pINHIBIT_US_FRONT;
		tFloat32 pINHIBIT_US_REAR;
		tFloat32 pNOTHING_US_FRONT;
		tFloat32 pNOTHING_US_REAR;
	} ControllerSettings;
	ControllerSettings set;

	int tWAIT_FOR_NEW_POSE;
	float tPOSE_REACHED_BEFORE;
	int myDesiredPoseTimer;
	int myActualPoseTimer;
	int cyclesFrontStop;
	int cyclesRearStop;


	//Active variables
	tFloat32 myRemoteSpeedSetpoint;
	tFloat32 myManeuverSpeedSetpoint;
	tFloat32 mySpeedSetpoint;
	tFloat32 myMeasSpeed;
	tFloat32 myOutputSpeed;
	tFloat32 myRemoteSteerSetpoint;



	//Controller I error buffer
	std::vector<tFloat64>::iterator myfloat64Iterator;
	std::deque<tFloat64> mErrors;
	int myI_BufferSize;


	tResult WriteOutXMLSpeed(tFloat32 vSpeed);
	tResult WriteOutXMLSteer(tFloat32 vData);

	bool myEstop;

	bool wL_myBrakeLights;
	bool wL_myRevLights;
	bool myHeadLightRequired;
	bool myHeadLightVal;
	bool myBrakeLightRequired;
	bool myBrakeLightVal;
	bool myRevLightRequired;
	bool myRevLightVal;

	bool myDebug1;
	bool myReadyToRun;



	tInt64 mySequenceTime;
	tInt64 myPreviousLocalTime;



	/*! the critical section of the timer setup */
	cCriticalSection m_oCriticalSectionTimerSetup;

	/*! media description for the output pin with speed */


	// IRunnable interface
public:
	tResult Run(tInt nActivationType, const tVoid *pvUserData, tInt nUserDataSize, IException **__exception_ptr);


	// cFilter interface

	OdometryPose getActualPose() const;

protected:
	tResult Stop(IException **__exception_ptr);


	// cFilter interface
protected:
	tResult Start(IException **__exception_ptr);

private:
	/*! holds the xml file for the supporting points*/
	cFilename m_fileConfig;

	/*! holds the yValues for the supporting points*/
	vector<tFloat32> m_yValues;
	/*! holds the xValues for the supporting points*/
	vector<tFloat32> m_xValues;

	/*! if debug console output is enabled */
	tBool m_bDebugModeEnabled;

	/*! indicates which calibration mode is used: 1: linear, 2: cubic, 3: none*/
	tInt m_iCalibrationMode;

	/*! reads the xml file which is set in the filter properties */
	tResult LoadConfigurationData();
	/*! checks the loaded configuration data; checks if the xvalues are in increasing order*/
	tResult CheckConfigurationData();

	/*! doing the linear interpolation
	@param fl32InputValue the value which should be interpolated
	*/
	tFloat32 getLinearInterpolatedValue(tFloat32 fl32InputValue);

	tResult WriteBoolToPin(cOutputPin *vOutputPin, bool vData);

	void InitializeData();








};


#endif
