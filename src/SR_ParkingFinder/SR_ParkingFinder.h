#ifndef _SR_ParkingFinder_H
#define _SR_ParkingFinder_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../misc/PinWrapper.h"
#include "../misc/SR_Logger.h"
#include <map_data.h>
#include <odometry_data.h>

using namespace std;
using namespace cv;
using namespace adtf;

#define OID_SR_ParkingFinder "sr.parking.finder"


class SR_ParkingFinder : public adtf::cFilter
{
        ADTF_FILTER(OID_SR_ParkingFinder, "SpaceRacer Parking Finder", adtf::OBJCAT_Tool)

public:
	SR_ParkingFinder(const tChar *info);
	virtual ~SR_ParkingFinder();
private:
	tResult final_calc(double dist);
private:
	enum Mode {
		Ready,
		GoToStart,
		FindStart,
		FindEnd
	} currMode;
	Point2d currPosition;
	OdometryPose currPose;
	Point2d startPoint;
	double distanceToStart;
	bool active;
    int counter;
	double currSlotLength;
	ParkingType typeToFind;
    tFloat32 dist_to_object;
	std::vector<float> uSonicFilter;
	double parallelSlotLength;
	double crossSlotLength;
	double depthLimit;
	SR_Logger logger;

protected:
	adtf::cInputPin searchRequestPin;
	WrappedSVOutputPin searchResultPin;
	adtf::cOutputPin referencePoseOut;
	WrappedBVOutputPin blinkerRequestPin;
	WrappedSVInputPin ultrasonic_input;
    adtf::cInputPin pose_input;
	WrappedBVInputPin abortPin;

	tResult Init(tInitStage stage, __exception);
	tResult Shutdown(tInitStage stage, __exception);

	tResult OnPinEvent(adtf::IPin *source, tInt eventCode, tInt param1, tInt param2, adtf::IMediaSample *mediaSample);
};


#endif
