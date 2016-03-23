#ifndef SR_IMAGE_PROCESSING_H_
#define SR_IMAGE_PROCESSING_H_

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <additional/adtf_signal_registry_support.h>
#include <adtf_graphics.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "IPM.h"
#include <map_data.h>
#include <odometry_data.h>
#include "../misc/PinWrapper.h"
#include "../misc/SR_Logger.h"

using namespace adtf;


#define OID_SR_IMAGEPROCESSING "sr.imageprocessing"

class ProcessingThread;

class SR_ImageProcessing : public adtf::cFilter
{
	ADTF_FILTER(OID_SR_IMAGEPROCESSING, "SpaceRacer Image Processing", adtf::OBJCAT_DataFilter)

public:
	enum OutputType {
		Threshold = 0,
		Canny = 1,
		Greyscale = 2,
		ReversedOrder = 3
	};

	typedef struct {
		OutputType outputType;

		bool debugPrintoutEnabled;
		bool debugCrossings;
		bool debugTimings;
		bool debugLaneFinder;
		bool debugDrivingPath;

		bool useAdaptiveThreshold;
		bool autoThreshold;
		int minContourArea;

		int dilationKernelW;
		int dilationKernelH;

		float laneCenterLength;
		float laneCenterDev;
		float laneCenterAsp;

		int rightLaneSearchX;
		int rightLaneSearchY;
		int leftLaneSearchX;
		int leftLaneSearchY;
		cv::Point2i leftLaneSearchCurveRightOffs;
		cv::Point2i leftLaneSearchCurveLeftOffs;

		int laneSmoothingSize;
		float laneSmoothingShape;
		int laneDetectorMaxDev;
		int laneExtrapolationDist;

		bool enableCrossingDetection;
		bool enableCrossingExitDetection;
		float crossingCornerLength1;
		float crossingCornerLength2;

		double pParkingWidth;
		double pParkingWidthDev;
		float pParkingSearchOffset;
        bool pParking;
		ParkingType pType;

		double meterPerPixel;
		double meterToTopDownCenter;
		std::vector<cv::Point2i> validImageRegion;
	} ImageProcPrefs;

	static volatile uint32_t thresholdValue;
	static SR_Logger logger;

	SR_ImageProcessing(const tChar *info);
	virtual ~SR_ImageProcessing();
	tResult PropertyChanged(const tChar *strName);
	tResult transmitTopDownImage(const cv::Mat &m);
	tResult transmitFilteredImage(const cv::Mat &m);
	tResult transmitThresholdImage(const cv::Mat &m);
	tResult transmitLaneData(const LaneData &ld);
	tResult transmitCrossingData(const CrossingDetectionEvent &event);
	tResult transmitParkingData(const ParkingSearchRequest &r);
	void setParking(const bool parking);

	bool shouldOutputTopDown() const;
	bool shouldOutputFiltered() const;
	bool shouldOutputThreshold() const;
	void applyHomography(cv::Mat *source, cv::Mat *target);
	ImageProcPrefs getPrefs() const;

protected:
	tResult Init(tInitStage stage, __exception);
	tResult Shutdown(tInitStage stage, __exception);
	tResult OnPinEvent(adtf::IPin *source, tInt eventCode, tInt param1, tInt param2, adtf::IMediaSample *mediaSample);
	tResult Start(IException **__exception_ptr);
	tResult Stop(IException **__exception_ptr);

private:
	// Video inputs
	adtf::cVideoPin VideoInput;
	adtf::cVideoPin DepthInput;

	// Video outputs
	adtf::cVideoPin filteredOutput;
	adtf::cVideoPin topDownOutput;
	adtf::cVideoPin thresholdOutput;

	WrappedBVInputPin searchParkingSlotPin;
	adtf::cInputPin posePin;
	adtf::cInputPin resetLanePin;

	adtf::cOutputPin crossingPin;
	WrappedBVOutputPin emergencyStopPin;
	adtf::cOutputPin parkingDistPin;
	adtf::cOutputPin laneDataPin;

	cMutex topDownMutex;
	cMutex threshMutex;
	cMutex filteredMutex;
	cMutex laneDataMutex;
	cMutex crossingDataMutex;
	cMutex parkingDataMutex;

	int numThreads;

	ImageProcPrefs prefs;
	mutable cMutex prefMutex;

	enum DetectedType {
		Garbage,
		Center,
		RightLaneCurve,
		LeftLaneCurve,
		Crossing
	};

	typedef struct
	{
		cv::RotatedRect rect;
		size_t contour_idx;
		DetectedType type;
		cv::Point2f curveCenter;
		cv::Point2f vecToCenter;
		double radiusEstimate;
		double cArea;
		double aspectRatio;
		float longSide;
		float shortSide;
	} Feature;

	typedef struct
	{
		cv::Point center;
		bool isBig;
	} CurveDescription;

	tResult transmitImage(adtf::cVideoPin *pin, const cv::Mat &m);
	tResult ProcessInput(IMediaSample* pSample, tTimeStamp);
	tResult ProcessDepth(IMediaSample* pSample);

	void recalcIPMMatrix();

	bool topDownConnected;
	bool filteredConnected;
	bool thresholdConnected;

	// Camera settings
	bool autoAngle;
	std::vector<double> camAngles;
	size_t camAngleIdx;
	double camAngle;
	double camOffsetY;
	double camOffsetZ;
	double camOffsetX;
	double topDownHeight;
	double topDownDist;
	double topDownF;

	// flag to check the first frame
    bool        m_bFirstFrame;
	int			currentFrame;
	bool		m_bFirstFrameDepth;

	// bitmap format of input pin
	tBitmapFormat inputFormatRGB;
	// bitmap format of output pin
	tBitmapFormat outputFormatGrey;

	tBitmapFormat outputFormatDepth;

	cv::Mat m_Intrinsics;
	cv::Mat m_Distorsion;
	cv::Mat m_IntrinsicsTopDown;

	IPM ipm;
	cMutex ipmMutex;

	std::vector<cv::Point2f> origPoints;
	std::vector<cv::Point2f> dstPoints;
	std::vector<cv::Point2f> gridPoints;

    bool parking;

	vector<ProcessingThread *> workers;
	vector<cKernelThread> workerThreads;

	OdometryPose currentPose;
};


#endif
