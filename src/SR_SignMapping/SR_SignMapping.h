#ifndef _SR_SIGNMAPPING_H
#define _SR_SIGNMAPPING_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <additional/adtf_signal_registry_support.h>
using namespace adtf;

#include <adtf_graphics.h>
using namespace adtf_graphics;

//#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>


#include "../misc/SR_Logger.h"
#include <odometry_data.h>

using namespace std;
using namespace cv;
#define OID_SR_SIGNMAPPING "sr.signmapping"


class SignMapping : public adtf::cFilter
{
	ADTF_FILTER(OID_SR_SIGNMAPPING, "SpaceRacer SignMapping", adtf::OBJCAT_DataFilter)

	public:
		SignMapping(const tChar *info);
	virtual ~SignMapping();
	SR_Logger logger;
private:
	tBool    m_firstTime;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionInput;
	tBufferID m_szIDRoadSignI16Identifier;
	tBufferID m_szIDRoadSignF32Imagesize;
	tBufferID m_szIDRoadSignExtAf32TVec;
	tBufferID m_szIDRoadSignExtAf32RVec;
	OdometryPose currentPose;
	Mat lastRotationPoint;
	Mat actRotationPoint;
	double camAngle;
	double camOffsetY;
	double camOffsetZ;
	double camOffsetX;
	double lastThetaRodrigues;
	bool lastSignDataOk;


protected:
	tResult Init(tInitStage stage, __exception);
	tResult Shutdown(tInitStage stage, __exception);

	tResult OnPinEvent(adtf::IPin *source, tInt eventCode, tInt param1, tInt param2, adtf::IMediaSample *mediaSample);
	void    ProcessInput();
protected:
	adtf::cInputPin signInput;
	adtf::cInputPin posePin;
	adtf::cOutputPin signOutput;

};


#endif
