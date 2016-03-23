#ifndef _SR_CARFOLLOWER_H
#define _SR_CARFOLLOWER_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <additional/adtf_signal_registry_support.h>
using namespace adtf;

#include <adtf_graphics.h>
using namespace adtf_graphics;

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../misc/PinWrapper.h"
#include "../misc/SR_Logger.h"
#include <odometry_data.h>
#include <map_data.h>
#include "../misc/LineHelpers.h"

using namespace std;
using namespace cv;
#define OID_SR_CARFOLLOWER "sr.carfollower"


class CarFollower : public adtf::cFilter
{
	ADTF_FILTER(OID_SR_CARFOLLOWER, "SpaceRacer CarFollower", adtf::OBJCAT_DataFilter)

public:
	CarFollower(const tChar *info);
	virtual ~CarFollower();
	SR_Logger logger;
	tHandle m_hTimer;
private:

	float commandVelocity;
	std::vector<float> filterRelativeTargetVelocity;
	std::vector<float> filterAbsoluteTargetVelocity;
	size_t filterRelativeTargetVelocityCounter;
	size_t filterAbsoluteTargetVelocityCounter;
	float targetRelativeVelocityFiltered;
	float targetAbsoluteVelocityFiltered;
	int signTimer;
	float lastRelativeDistance;
	float lastAbsoluteDistance;
	tTimeStamp lastTimeStamp;
	tBool m_firstTime;
	tBool debug1;
	tBool followingActive;
	tBool lastFollowingActive;
	OdometryPose currentPose;
	OdometryPose targetPose;
	OdometryPose lastTargetPose;
	tResult processSigns(SignStruct &data);
	void enableFollowing();
	void disableFollowing();

	void handleKeyboardCommand(float command);

	cCriticalSection m_oCriticalSectionTimerSetup;
	tResult createTimer();
	tResult destroyTimer(ucom::IException **__exception_ptr);

	tResult sendDesiredPose(const ControllerCommand &p);
protected:
	tResult Init(tInitStage stage, __exception);
	tResult Shutdown(tInitStage stage, __exception);

	tResult OnPinEvent(adtf::IPin *source, tInt eventCode, tInt param1, tInt param2, adtf::IMediaSample *mediaSample);
    void    ProcessInput();
protected:
	adtf::cInputPin posePin;
	WrappedSVInputPin keyboardCommandPin;
	adtf::cInputPin signInputPin;
	WrappedBVInputPin controllerPoseReachedPin;

	adtf::cOutputPin controllerOutputPin;


	// IRunnable interface
public:
	tResult Run(tInt nActivationType, const tVoid *pvUserData, tInt nUserDataSize, IException **__exception_ptr);

	// cFilter interface
protected:
	tResult Start(IException **__exception_ptr);
	tResult Stop(IException **__exception_ptr);
};


#endif
