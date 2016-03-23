#include "ProcessingThread.h"

#include <adtf_utils.h>

#include <stdlib.h>
#include <cmath>
#include <ctime>
#include <mutexlocker.h>

using namespace std;
using namespace cv;
using namespace adtf;
using namespace adtf_graphics;

// Create filter shell
#define PROP_FILE "Camera::Camera calibration file"
#define PROP_ALPHA "Camera::Camera Angle [deg]"
#define PROP_CAM_OFFSET_Y "Camera::Camera height [m]"
#define PROP_CAM_OFFSET_Z "Camera::Camera distance [m]"
#define PROP_CAM_OFFSET_X "Camera::Camera side offset [m]"
#define PROP_AUTO_ANGLE "Camera::Camera Angle From Depth Sensor"

#define PROP_TOP_DOWN_HEIGHT "TopDown::Top down height [m]"
#define PROP_TOP_DOWN_DIST "TopDown::Top down distance [m]"
#define PROP_TOP_DOWN_FOV "TopDown::Top down focal length [pix]"
#define PROP_TOP_DOWN_FAC "TopDown::Pixel To Meter Correction Factor"

#define PROP_THRESHOLD "Filter::Canny Threshold"
#define PROP_OUTPUT_TYPE "Filter::Output Type"
#define PROP_THREDHOLD_AREA_DISCARD "Filter::Canny min area size [pix]"
#define PROP_KERN_W	"Filter::Dilation Kernel Width"
#define PROP_KERN_H	"Filter::Dilation Kernel Height"
#define PROP_THRESHOLD_ADAPTIVE "Filter::Use adaptive threshold"
#define PROP_AUTOEXPOSURE "Filter::Automatically adjust threshold"
#define PROP_NUMTHREADS "Filter::Number of parallel threads"

#define PROP_DEBUGPRINTS "Debug::General Debug Spam"
#define PROP_CROSSING_DEBUG	"Debug::Debug Crossings"
#define PROP_DEBUG_FRAMETIMES "Debug::Show Frametimings"
#define PROP_DEBUG_LANEFINDER "Debug::Lane Finder"
#define PROP_DEBUG_DRIVINGPATH "Debug::Simulate Driving Path"

#define PROP_LANE_CENTER_L	"Lanes::Center::Length"
#define PROP_LANE_CENTER_D	"Lanes::Center::Dev"
#define PROP_LANE_CENTER_A	"Lanes::Center::Aspect"
#define PROP_CROSSING_ENABLED "Lanes::Crossing::Enable crossing detection"
#define PROP_CROSSING_EXITS	"Lanes::Crossing::Enable crossing exit detection"
#define PROP_CROSSING_CORNERL1 "Lanes::Crossing::Corner length 1"
#define PROP_CROSSING_CORNERL2 "Lanes::Crossing::Corner length 2"
#define PROP_DETECTOR_SMOOTHINGSIZE "Lanes::Detector::SmoothingSize"
#define PROP_DETECTOR_SMOOTHINGSHAPE "Lanes::Detector::SmoothingShape"
#define PROP_DETECTOR_MAXDEV "Lanes::Detector::Max Deviation from old lane [pix]"
#define PROP_DETECTOR_LANEEXTRAPDIST "Lanes::Detector::Extrapolation distance [pix]"
#define PROP_DETECTOR_RX "Lanes::Detector::Right start x"
#define PROP_DETECTOR_RY "Lanes::Detector::Right start y"
#define PROP_DETECTOR_LX "Lanes::Detector::Left start x"
#define PROP_DETECTOR_LY "Lanes::Detector::Left start y"
#define PROP_DETECTOR_OFFS_CRX "Lanes::Detector::Left Lane Curve Right offs X [pix]"
#define PROP_DETECTOR_OFFS_CRY "Lanes::Detector::Left Lane Curve Right offs y [pix]"
#define PROP_DETECTOR_OFFS_CLX "Lanes::Detector::Left Lane Curve Left offs X [pix]"
#define PROP_DETECTOR_OFFS_CLY "Lanes::Detector::Left Lane Curve Left offs y [pix]"

#define PROP_PPARKING_WIDTH	"Parking::Parallel::Rect width"
#define PROP_PPARKING_WIDTH_DEV	"Parking::Parallel::Rect width max deviation"
#define PROP_PARKING_OFFSET	"Parking::Search Offset"

#define INIT_PROP(type, prop, field, val) (field) = (val); type((prop), (field));
#define INIT_PROP_INT(prop, field, val) INIT_PROP(SetPropertyInt, (prop), (field), (val));
#define INIT_PROP_INT_DYN(prop, field, val) INIT_PROP_INT((prop), (field), (val)); SetPropertyBool(prop NSSUBPROP_ISCHANGEABLE, tTrue);

ADTF_FILTER_PLUGIN("SpaceRacer Image Processing", OID_SR_IMAGEPROCESSING, SR_ImageProcessing)

volatile uint32_t SR_ImageProcessing::thresholdValue = 100;
SR_Logger SR_ImageProcessing::logger(OID_SR_IMAGEPROCESSING, "../../../../aadc_log");

SR_ImageProcessing::SR_ImageProcessing(const tChar *info) : cFilter(info)
{
	topDownConnected = false;
	filteredConnected = false;
	thresholdConnected = false;

	prefs.outputType = Threshold;

	prefs.debugPrintoutEnabled = false;
	prefs.debugCrossings = false;
	prefs.debugTimings = false;
	prefs.debugLaneFinder = false;
	prefs.debugDrivingPath = false;

	prefs.useAdaptiveThreshold = false;
	prefs.autoThreshold = true;
	prefs.minContourArea = 100;

	prefs.dilationKernelW = 5;
	prefs.dilationKernelH = 5;

	prefs.laneCenterLength = .3f;
	prefs.laneCenterDev = .08f;
	prefs.laneCenterAsp = 3.5;

	prefs.rightLaneSearchX = 320;
	prefs.rightLaneSearchY = 475;
	prefs.leftLaneSearchX = 320;
	prefs.leftLaneSearchY = 420;
	prefs.leftLaneSearchCurveRightOffs = Point2i(20, 20);
	prefs.leftLaneSearchCurveLeftOffs = Point2i(-20, -20);

	prefs.laneSmoothingSize = 4;
	prefs.laneSmoothingShape = 0;
	prefs.laneDetectorMaxDev = 16;

	prefs.enableCrossingDetection = true;
	prefs.enableCrossingExitDetection = true;
	prefs.crossingCornerLength1 = .2;
	prefs.crossingCornerLength2 = .4;

	prefs.pParkingWidth = .7;
	prefs.pParkingWidthDev = .1;

	prefs.meterPerPixel = 1;
	prefs.meterToTopDownCenter = 1;

	camAngle = 24.5;
	camOffsetY = .3;
	camOffsetZ = .21;
	camOffsetX = .025;
	topDownHeight = 4;
	topDownDist = 2.5;
	topDownF = 550;
	camAngleIdx = 0;
	autoAngle = false;
	currentFrame = 0;

	SetPropertyStr(PROP_FILE,"");
	SetPropertyBool(PROP_FILE NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr(PROP_FILE NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "YML Files (*.yml)");
	SetPropertyStr(PROP_FILE NSSUBPROP_DESCRIPTION, "Here you have to set the file with calibration paraemters of the used camera");
	SetPropertyInt(PROP_THRESHOLD, SR_ImageProcessing::thresholdValue);
	SetPropertyBool(PROP_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_THRESHOLD NSSUBPROP_DESCRIPTION, "The threshold value for canny to detect lines.");
	SetPropertyInt(PROP_THRESHOLD NSSUBPROP_MAX, 255);
	SetPropertyInt(PROP_THRESHOLD NSSUBPROP_MIN, 0);
	m_Intrinsics = (Mat_<double>(3,3) << 550, 0, 320, 0, 550, 240, 0, 0, 1);
	m_IntrinsicsTopDown = (Mat_<double>(3,3) << topDownF, 0, 320, 0, topDownF, 240, 0, 0, 1);
	SetPropertyFloat(PROP_ALPHA, camAngle);
	SetPropertyBool(PROP_ALPHA NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyFloat(PROP_CAM_OFFSET_Y, camOffsetY);
	SetPropertyBool(PROP_CAM_OFFSET_Y NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyFloat(PROP_CAM_OFFSET_Z, camOffsetZ);
	SetPropertyBool(PROP_CAM_OFFSET_Z NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyFloat(PROP_CAM_OFFSET_X, camOffsetX);
	SetPropertyBool(PROP_CAM_OFFSET_X NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyFloat(PROP_TOP_DOWN_HEIGHT, topDownHeight);
	SetPropertyBool(PROP_TOP_DOWN_HEIGHT NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyFloat(PROP_TOP_DOWN_DIST, topDownDist);
	SetPropertyBool(PROP_TOP_DOWN_DIST NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyFloat(PROP_TOP_DOWN_FOV, topDownF);
	SetPropertyBool(PROP_TOP_DOWN_FOV NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(PROP_OUTPUT_TYPE, prefs.outputType);
	SetPropertyStr(PROP_OUTPUT_TYPE NSSUBPROP_VALUELIST, "0@Threshold Filter|1@Canny|2@Greyscale|3@ReversedOrder");
	SetPropertyBool(PROP_OUTPUT_TYPE NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat(PROP_LANE_CENTER_L, prefs.laneCenterLength);
	SetPropertyStr(PROP_LANE_CENTER_L NSSUBPROP_DESCRIPTION, "In meters");
	SetPropertyBool(PROP_LANE_CENTER_L NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat(PROP_LANE_CENTER_D, prefs.laneCenterDev);
	SetPropertyStr(PROP_LANE_CENTER_D NSSUBPROP_DESCRIPTION, "In meters");
	SetPropertyBool(PROP_LANE_CENTER_D NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat(PROP_LANE_CENTER_A, prefs.laneCenterAsp);
	SetPropertyStr(PROP_LANE_CENTER_A NSSUBPROP_DESCRIPTION, "Min aspect ratio");
	SetPropertyBool(PROP_LANE_CENTER_A NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt(PROP_DETECTOR_RX, prefs.rightLaneSearchX);
	SetPropertyInt(PROP_DETECTOR_RY, prefs.rightLaneSearchY);
	SetPropertyInt(PROP_DETECTOR_LX, prefs.leftLaneSearchX);
	SetPropertyInt(PROP_DETECTOR_LY, prefs.leftLaneSearchY);
	SetPropertyBool(PROP_DETECTOR_RX NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_DETECTOR_RY NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_DETECTOR_LX NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_DETECTOR_LY NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt(PROP_THREDHOLD_AREA_DISCARD, prefs.minContourArea);
	SetPropertyBool(PROP_THREDHOLD_AREA_DISCARD NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyBool(PROP_DEBUGPRINTS, prefs.debugPrintoutEnabled);
	SetPropertyBool(PROP_DEBUGPRINTS NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyBool(PROP_DEBUG_LANEFINDER, prefs.debugLaneFinder);
	SetPropertyBool(PROP_DEBUG_LANEFINDER NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyBool(PROP_DEBUG_DRIVINGPATH, prefs.debugDrivingPath);
	SetPropertyBool(PROP_DEBUG_DRIVINGPATH NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyBool(PROP_THRESHOLD_ADAPTIVE, prefs.useAdaptiveThreshold);
	SetPropertyBool(PROP_THRESHOLD_ADAPTIVE NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt(PROP_DETECTOR_SMOOTHINGSIZE, prefs.laneSmoothingSize);
	SetPropertyInt(PROP_DETECTOR_SMOOTHINGSIZE NSSUBPROP_MIN, 1);
	SetPropertyBool(PROP_DETECTOR_SMOOTHINGSIZE NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat(PROP_DETECTOR_SMOOTHINGSHAPE, prefs.laneSmoothingShape);
	SetPropertyFloat(PROP_DETECTOR_SMOOTHINGSHAPE NSSUBPROP_MIN, 0);
	SetPropertyFloat(PROP_DETECTOR_SMOOTHINGSHAPE NSSUBPROP_MAX, 1);
	SetPropertyBool(PROP_DETECTOR_SMOOTHINGSHAPE NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt(PROP_DETECTOR_MAXDEV, prefs.laneDetectorMaxDev);
	SetPropertyInt(PROP_DETECTOR_MAXDEV NSSUBPROP_MIN, 1);
	SetPropertyInt(PROP_DETECTOR_MAXDEV NSSUBPROP_MAX, 64);
	SetPropertyBool(PROP_DETECTOR_MAXDEV NSSUBPROP_ISCHANGEABLE, tTrue);

	INIT_PROP_INT_DYN(PROP_DETECTOR_LANEEXTRAPDIST, prefs.laneExtrapolationDist, 32)

	SetPropertyInt(PROP_KERN_H, prefs.dilationKernelH);
	SetPropertyInt(PROP_KERN_W, prefs.dilationKernelW);
	SetPropertyBool(PROP_KERN_H NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_KERN_W NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(PROP_KERN_H NSSUBPROP_MIN, 0);
	SetPropertyInt(PROP_KERN_W NSSUBPROP_MIN, 0);

	SetPropertyBool(PROP_AUTOEXPOSURE, prefs.autoThreshold);
	SetPropertyBool(PROP_AUTOEXPOSURE NSSUBPROP_ISCHANGEABLE, tTrue);

	autoAngle = false;
	SetPropertyBool(PROP_AUTO_ANGLE, autoAngle);
	SetPropertyBool(PROP_AUTO_ANGLE NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat(PROP_PPARKING_WIDTH, prefs.pParkingWidth);
	SetPropertyFloat(PROP_PPARKING_WIDTH_DEV, prefs.pParkingWidthDev);
	SetPropertyBool(PROP_PPARKING_WIDTH NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_PPARKING_WIDTH_DEV NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyBool(PROP_CROSSING_DEBUG, prefs.debugCrossings);
	SetPropertyBool(PROP_CROSSING_DEBUG NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyBool(PROP_CROSSING_ENABLED, prefs.enableCrossingDetection);
	SetPropertyBool(PROP_CROSSING_EXITS, prefs.enableCrossingExitDetection);

	SetPropertyFloat(PROP_CROSSING_CORNERL1, prefs.crossingCornerLength1);
	SetPropertyFloat(PROP_CROSSING_CORNERL2, prefs.crossingCornerLength2);
	SetPropertyBool(PROP_CROSSING_CORNERL1 NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_CROSSING_CORNERL2 NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyBool(PROP_DEBUG_FRAMETIMES, prefs.debugTimings);
	SetPropertyBool(PROP_DEBUG_FRAMETIMES NSSUBPROP_ISCHANGEABLE, tTrue);

	prefs.pParkingSearchOffset = .02;
	SetPropertyFloat(PROP_PARKING_OFFSET, prefs.pParkingSearchOffset);
	SetPropertyBool(PROP_PARKING_OFFSET NSSUBPROP_ISCHANGEABLE, tTrue);

    prefs.pParking = false;

	numThreads = 2;
	SetPropertyInt(PROP_NUMTHREADS, numThreads);
	SetPropertyInt(PROP_NUMTHREADS NSSUBPROP_MIN, 1);
	SetPropertyInt(PROP_NUMTHREADS NSSUBPROP_MAX, 4);

	SetPropertyInt(PROP_DETECTOR_OFFS_CRX, prefs.leftLaneSearchCurveRightOffs.x);
	SetPropertyInt(PROP_DETECTOR_OFFS_CRY, prefs.leftLaneSearchCurveRightOffs.y);
	SetPropertyInt(PROP_DETECTOR_OFFS_CLX, prefs.leftLaneSearchCurveLeftOffs.x);
	SetPropertyInt(PROP_DETECTOR_OFFS_CLY, prefs.leftLaneSearchCurveLeftOffs.y);
	SetPropertyBool(PROP_DETECTOR_OFFS_CRX NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_DETECTOR_OFFS_CRY NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_DETECTOR_OFFS_CLX NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_DETECTOR_OFFS_CLY NSSUBPROP_ISCHANGEABLE, tTrue);
}

SR_ImageProcessing::~SR_ImageProcessing()
{
	for (size_t i = 0; i < workers.size(); ++i) {
		delete workers[i];
	}
}

tResult SR_ImageProcessing::Init(adtf::cFilter::tInitStage stage, ucom::IException **__exception_ptr)
{

	// call base implementation
	RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))

	if (stage == StageFirst) {
		RETURN_IF_FAILED(VideoInput.Create("RGB_Input",IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&VideoInput));

		RETURN_IF_FAILED(DepthInput.Create("Depth_Image", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&DepthInput));

		RETURN_IF_FAILED(filteredOutput.Create("FilteredImage", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&filteredOutput));

		RETURN_IF_FAILED(topDownOutput.Create("TopDownImage", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&topDownOutput));

		RETURN_IF_FAILED(thresholdOutput.Create("ThresholdImage", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&thresholdOutput));

        REGISTER_MEDIA_PIN(parkingDistPin, MEDIA_TYPE_MAPDATA, MEDIA_SUBTYPE_MAPDATA_PARKING_SEARCH_REQ, "Distance_to_Parking");

		REGISTER_MEDIA_PIN(crossingPin, MEDIA_TYPE_MAPDATA, MEDIA_SUBTYPE_MAPDATA_CROSSING_EVENT, "Crossing_Detected");

		REGISTER_MEDIA_PIN(posePin, MEDIA_TYPE_ODOMETRY, MEDIA_SUBTYPE_ODOMETRY_POSE, "Current_Pose");

		REGISTER_MEDIA_PIN(resetLanePin, MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_UINT8, "Reset_Lane_Tracking");

		INIT_WRAPPED_PIN(emergencyStopPin, "Emergency_Stop");
		INIT_WRAPPED_PIN(searchParkingSlotPin, "Search_Parkingslot");

		REGISTER_MEDIA_PIN(laneDataPin, MEDIA_TYPE_MAPDATA, MEDIA_SUBTYPE_MAPDATA_LANEDATA, "Lane_Data");
	} else if (stage == StageNormal) {
		m_bFirstFrame = true;
		m_bFirstFrameDepth = true;
		SR_ImageProcessing::thresholdValue = GetPropertyInt(PROP_THRESHOLD);

		//Get path of calibration file with camera paramters
		cFilename fileCalibration = GetPropertyStr(PROP_FILE); ;

		//Get path of calibration file with camera paramters
		ADTF_GET_CONFIG_FILENAME(fileCalibration);
		fileCalibration = fileCalibration.CreateAbsolutePath(".");
		//check if calibration file with camera paramters exits
		if (fileCalibration.IsEmpty() || !(cFileSystem::Exists(fileCalibration))) {
			LOG_ERROR("Calibration File for camera not found");
		} else {
			// read the calibration file with camera paramters exits and save to member variable
			cv::FileStorage camera_data (fileCalibration.GetPtr(),cv::FileStorage::READ);
			camera_data ["camera_matrix"] >> m_Intrinsics;
			camera_data ["distortion_coefficients"] >> m_Distorsion;
			logger.log() << m_Intrinsics << endl << m_Distorsion << endl;
		}

		camAngle = GetPropertyFloat(PROP_ALPHA, camAngle);
        camOffsetY = GetPropertyFloat(PROP_CAM_OFFSET_Y, camOffsetY
);
		camOffsetZ = GetPropertyFloat(PROP_CAM_OFFSET_Z, camOffsetZ);
		camOffsetX = GetPropertyFloat(PROP_CAM_OFFSET_X, camOffsetX);
		topDownHeight = GetPropertyFloat(PROP_TOP_DOWN_HEIGHT, topDownHeight);
		topDownDist = GetPropertyFloat(PROP_TOP_DOWN_DIST, topDownDist);
		topDownF = GetPropertyFloat(PROP_TOP_DOWN_FOV, topDownF);
		m_IntrinsicsTopDown = (Mat_<double>(3,3) << topDownF, 0, 320, 0, topDownF, 240, 0, 0, 1);
		prefs.laneCenterLength = GetPropertyFloat(PROP_LANE_CENTER_L, prefs.laneCenterLength);
		prefs.laneCenterDev = GetPropertyFloat(PROP_LANE_CENTER_D, prefs.laneCenterDev);
        prefs.laneCenterAsp = GetPropertyFloat(PROP_LANE_CENTER_A, prefs.laneCenterAsp);
		prefs.minContourArea = GetPropertyInt(PROP_THREDHOLD_AREA_DISCARD, prefs.minContourArea);
		prefs.laneSmoothingSize = GetPropertyInt(PROP_DETECTOR_SMOOTHINGSIZE, prefs.laneSmoothingSize);
		prefs.laneSmoothingShape = GetPropertyFloat(PROP_DETECTOR_SMOOTHINGSHAPE, prefs.laneSmoothingShape);
		prefs.laneDetectorMaxDev = GetPropertyInt(PROP_DETECTOR_MAXDEV, prefs.laneDetectorMaxDev);
		prefs.laneExtrapolationDist = GetPropertyInt(PROP_DETECTOR_LANEEXTRAPDIST, prefs.laneExtrapolationDist);

		prefs.rightLaneSearchX = GetPropertyInt(PROP_DETECTOR_RX, prefs.rightLaneSearchX);
		prefs.rightLaneSearchY = GetPropertyInt(PROP_DETECTOR_RY, prefs.rightLaneSearchY);
		prefs.leftLaneSearchX = GetPropertyInt(PROP_DETECTOR_LX, prefs.leftLaneSearchX);
		prefs.leftLaneSearchY = GetPropertyInt(PROP_DETECTOR_LY, prefs.leftLaneSearchY);

		prefs.leftLaneSearchCurveRightOffs.x = GetPropertyInt(PROP_DETECTOR_OFFS_CRX, prefs.leftLaneSearchCurveRightOffs.x);
		prefs.leftLaneSearchCurveRightOffs.y = GetPropertyInt(PROP_DETECTOR_OFFS_CRY, prefs.leftLaneSearchCurveRightOffs.y);
		prefs.leftLaneSearchCurveLeftOffs.x = GetPropertyInt(PROP_DETECTOR_OFFS_CLX, prefs.leftLaneSearchCurveLeftOffs.x);
		prefs.leftLaneSearchCurveLeftOffs.y = GetPropertyInt(PROP_DETECTOR_OFFS_CLY, prefs.leftLaneSearchCurveLeftOffs.y);

		prefs.dilationKernelH = GetPropertyInt(PROP_KERN_H, prefs.dilationKernelH);
		prefs.dilationKernelW = GetPropertyInt(PROP_KERN_W, prefs.dilationKernelW);

		prefs.debugPrintoutEnabled = GetPropertyBool(PROP_DEBUGPRINTS, prefs.debugPrintoutEnabled);
		prefs.useAdaptiveThreshold = GetPropertyBool(PROP_THRESHOLD_ADAPTIVE, prefs.useAdaptiveThreshold);
		prefs.debugLaneFinder = GetPropertyBool(PROP_DEBUG_LANEFINDER, prefs.debugLaneFinder);
		prefs.debugDrivingPath = GetPropertyBool(PROP_DEBUG_DRIVINGPATH, prefs.debugDrivingPath);
		prefs.autoThreshold = GetPropertyBool(PROP_AUTOEXPOSURE, prefs.autoThreshold);
		autoAngle = GetPropertyBool(PROP_AUTO_ANGLE, autoAngle);

		prefs.outputType = (SR_ImageProcessing::OutputType)GetPropertyInt(PROP_OUTPUT_TYPE, prefs.outputType);
		recalcIPMMatrix();

		prefs.pParkingWidth = GetPropertyFloat(PROP_PPARKING_WIDTH, prefs.pParkingWidth);
		prefs.pParkingWidthDev = GetPropertyFloat(PROP_PPARKING_WIDTH_DEV, prefs.pParkingWidthDev);
		prefs.debugCrossings = GetPropertyBool(PROP_CROSSING_DEBUG, prefs.debugCrossings);
		prefs.enableCrossingDetection = GetPropertyBool(PROP_CROSSING_ENABLED, prefs.enableCrossingDetection);
		prefs.enableCrossingExitDetection = GetPropertyBool(PROP_CROSSING_EXITS, prefs.enableCrossingExitDetection);
		prefs.crossingCornerLength1 = GetPropertyFloat(PROP_CROSSING_CORNERL1, prefs.crossingCornerLength1);
		prefs.crossingCornerLength2 = GetPropertyFloat(PROP_CROSSING_CORNERL2, prefs.crossingCornerLength2);

		prefs.debugTimings = GetPropertyBool(PROP_DEBUG_FRAMETIMES, prefs.debugTimings);

		prefs.pParkingSearchOffset = GetPropertyFloat(PROP_PARKING_OFFSET, prefs.pParkingSearchOffset);
	} else if (stage == StageGraphReady) {
		topDownConnected = topDownOutput.IsConnected();
		filteredConnected = filteredOutput.IsConnected();
		thresholdConnected = thresholdOutput.IsConnected();
		numThreads = GetPropertyInt(PROP_NUMTHREADS, numThreads);
		for (int i = 0; i < numThreads; ++i) {
			cString name = cString::Format("Worker %d", i + 1);
			workers.push_back(new ProcessingThread(name.GetPtr(), this));
		}
		for (size_t i = 0; i < workers.size(); ++i) {
			RETURN_IF_FAILED(workers[i]->createThread());
		}
		/*
		workers.back()->testExtrapolation();
		// stop after test
		return -1;
		*/
	}
	RETURN_NOERROR;
}

tResult SR_ImageProcessing::Shutdown(adtf::cFilter::tInitStage stage, ucom::IException **__exception_ptr)
{
	if (stage == StageGraphReady) {
		for (size_t i = 0; i < workers.size(); ++i) {
			RETURN_IF_FAILED(workers[i]->stopThread());
			RETURN_IF_FAILED(workers[i]->shutdownThread());
			delete workers[i];
		}
		workers.clear();
	} else if (stage == StageNormal) {

	} else if (stage == StageFirst) {

	}

	// call base implementation
	return cFilter::Shutdown(stage, __exception_ptr);
}

tResult SR_ImageProcessing::OnPinEvent(adtf::IPin *source, tInt eventCode, tInt param1, tInt param2, adtf::IMediaSample *mediaSample)
{
	if (eventCode == IPinEventSink::PE_MediaSampleReceived) {
		RETURN_IF_POINTER_NULL(mediaSample);
		tTimeStamp InputTimeStamp;
		InputTimeStamp = mediaSample->GetTime();
		if (source == &VideoInput) {
			if (m_bFirstFrame)
			{
				cObjectPtr<IMediaType> pType;
				RETURN_IF_FAILED(VideoInput.GetMediaType(&pType));
				cObjectPtr<IMediaTypeVideo> pTypeVideo;
				RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
				const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
				if (pFormat == NULL)
				{
					LOG_ERROR("No Bitmap information found on pin \"input\"");
					RETURN_ERROR(ERR_NOT_SUPPORTED);
				}
				inputFormatRGB.nPixelFormat = pFormat->nPixelFormat;
				inputFormatRGB.nWidth = pFormat->nWidth;
				inputFormatRGB.nHeight =  pFormat->nHeight;
				inputFormatRGB.nBitsPerPixel = pFormat->nBitsPerPixel;
				inputFormatRGB.nBytesPerLine = pFormat->nBytesPerLine;
				inputFormatRGB.nSize = pFormat->nSize;
				inputFormatRGB.nPaletteSize = pFormat->nPaletteSize;
				logger.log() << "pixel format " << pFormat->nPixelFormat << " with: " << pFormat->nWidth << " height: "
							 << pFormat->nHeight << " bpp " << pFormat->nBitsPerPixel << " bpl " << pFormat->nBytesPerLine
							 << " size " << pFormat->nSize << " palette size " << pFormat->nPaletteSize << endl;
				outputFormatGrey = inputFormatRGB;
				outputFormatGrey.nBitsPerPixel = 8;
				outputFormatGrey.nBytesPerLine = inputFormatRGB.nBytesPerLine / 3;
				outputFormatGrey.nSize = inputFormatRGB.nSize / 3;
				filteredOutput.SetFormat(&outputFormatGrey, NULL);
				m_bFirstFrame = false;
				topDownOutput.SetFormat(&inputFormatRGB, NULL);
				thresholdOutput.SetFormat(&outputFormatGrey, NULL);
			}
			currentFrame++;
			return ProcessInput(mediaSample,InputTimeStamp);
		} else if (source == &DepthInput && autoAngle) {
			if (m_bFirstFrameDepth) {
				m_bFirstFrameDepth = false;
				cObjectPtr<IMediaType> pType;
				RETURN_IF_FAILED(DepthInput.GetMediaType(&pType));
				cObjectPtr<IMediaTypeVideo> pTypeVideo;
				RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
				const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
				if (pFormat == NULL)

				{
					LOG_ERROR("No Bitmap information found on pin \"input\"");
					RETURN_ERROR(ERR_NOT_SUPPORTED);
				}
				outputFormatDepth = *pFormat;
			}
			return ProcessDepth(mediaSample);
		} else if (source == searchParkingSlotPin.ppin){
            prefs.pParking = true;
			bool type = false;
			searchParkingSlotPin.getValue(mediaSample, &type);
			prefs.pType = type ? ParkingTypeParallel : ParkingTypeCross;
		} else if (source == &posePin) {
			READ_MEDIA_PIN(mediaSample, OdometryPose, currentPose);
		} else if (source == &resetLanePin) {
			for (size_t i = 0; i < workers.size(); ++i) {
				workers[i]->resetLaneTracking();
			}
		}
	}

	RETURN_NOERROR;
}

tResult SR_ImageProcessing::Start(IException **__exception_ptr)
{
	for (size_t i = 0; i < workers.size(); ++i) {
		RETURN_IF_FAILED(workers[i]->runThread());
	}
	RETURN_NOERROR;
}

tResult SR_ImageProcessing::Stop(IException **__exception_ptr)
{
	for (size_t i = 0; i < workers.size(); ++i) {
		RETURN_IF_FAILED(workers[i]->stopThread());
	}
	RETURN_NOERROR;
}

tResult SR_ImageProcessing::PropertyChanged(const tChar *strName)
{
    MutexLocker ml(&prefMutex);
    if (cString::IsEqual(strName, PROP_THRESHOLD)) {
        SR_ImageProcessing::thresholdValue = GetPropertyInt(PROP_THRESHOLD);
    } else if (cString::IsEqual(strName, PROP_ALPHA)) {
        camAngle = GetPropertyFloat(PROP_ALPHA, camAngle);
		recalcIPMMatrix();
	} else if (cString::IsEqual(strName, PROP_CAM_OFFSET_Z)) {
		camOffsetZ = GetPropertyFloat(PROP_CAM_OFFSET_Z, camOffsetZ);
		recalcIPMMatrix();
	} else if (cString::IsEqual(strName, PROP_CAM_OFFSET_Y)) {
		camOffsetY = GetPropertyFloat(PROP_CAM_OFFSET_Y, camOffsetY);
		recalcIPMMatrix();
	} else if (cString::IsEqual(strName, PROP_CAM_OFFSET_X)) {
		camOffsetX = GetPropertyFloat(PROP_CAM_OFFSET_X, camOffsetX);
		recalcIPMMatrix();
	} else if (cString::IsEqual(strName, PROP_TOP_DOWN_HEIGHT)) {
		topDownHeight = GetPropertyFloat(PROP_TOP_DOWN_HEIGHT, topDownHeight);
		recalcIPMMatrix();
	} else if (cString::IsEqual(strName, PROP_TOP_DOWN_DIST)) {
		topDownDist = GetPropertyFloat(PROP_TOP_DOWN_DIST, topDownDist);
		recalcIPMMatrix();
	} else if (cString::IsEqual(strName, PROP_TOP_DOWN_FOV)) {
		topDownF = GetPropertyFloat(PROP_TOP_DOWN_FOV, topDownF);
		// Mutex this?
        m_IntrinsicsTopDown = (Mat_<double>(3,3) << topDownF, 0, 320, 0, topDownF, 240, 0, 0, 1);
		recalcIPMMatrix();
	} else if (cString::IsEqual(strName, PROP_LANE_CENTER_L)) {
		prefs.laneCenterLength = GetPropertyFloat(PROP_LANE_CENTER_L);
	} else if (cString::IsEqual(strName, PROP_LANE_CENTER_D)) {
		prefs.laneCenterDev = GetPropertyFloat(PROP_LANE_CENTER_D);
	} else if (cString::IsEqual(strName, PROP_LANE_CENTER_A)) {
		prefs.laneCenterAsp = GetPropertyFloat(PROP_LANE_CENTER_A);
	} else if (cString::IsEqual(strName, PROP_DETECTOR_SMOOTHINGSIZE)) {
		prefs.laneSmoothingSize = GetPropertyInt(PROP_DETECTOR_SMOOTHINGSIZE, prefs.laneSmoothingSize);
	} else if (cString::IsEqual(strName, PROP_DETECTOR_SMOOTHINGSHAPE)) {
		prefs.laneSmoothingShape = GetPropertyFloat(PROP_DETECTOR_SMOOTHINGSHAPE, prefs.laneSmoothingShape);
	} else if (cString::IsEqual(strName, PROP_DETECTOR_MAXDEV)) {
		prefs.laneDetectorMaxDev = GetPropertyFloat(PROP_DETECTOR_MAXDEV, prefs.laneDetectorMaxDev);
	} else if (cString::IsEqual(strName, PROP_DETECTOR_LANEEXTRAPDIST)) {
		prefs.laneExtrapolationDist = GetPropertyFloat(PROP_DETECTOR_LANEEXTRAPDIST);
	} else if (cString::IsEqual(strName, PROP_THREDHOLD_AREA_DISCARD)) {
		prefs.minContourArea = GetPropertyInt(PROP_THREDHOLD_AREA_DISCARD);
	} else if (cString::IsEqual(strName, PROP_KERN_H)) {
		prefs.dilationKernelH = GetPropertyInt(PROP_KERN_H);
	} else if (cString::IsEqual(strName, PROP_KERN_W)) {
		prefs.dilationKernelW = GetPropertyInt(PROP_KERN_W);
	} else if (cString::IsEqual(strName, PROP_OUTPUT_TYPE)) {
		prefs.outputType = (SR_ImageProcessing::OutputType)GetPropertyInt(PROP_OUTPUT_TYPE);
	} else if (cString::IsEqual(strName, PROP_DEBUGPRINTS)) {
		prefs.debugPrintoutEnabled = GetPropertyBool(PROP_DEBUGPRINTS);
	} else if (cString::IsEqual(strName, PROP_THRESHOLD_ADAPTIVE)) {
		prefs.useAdaptiveThreshold = GetPropertyBool(PROP_THRESHOLD_ADAPTIVE);
	} else if (cString::IsEqual(strName, PROP_AUTOEXPOSURE)) {
		prefs.autoThreshold = GetPropertyBool(PROP_AUTOEXPOSURE);
		if (!prefs.autoThreshold) {
			// Reset thresh to whatever was set by hand
			SR_ImageProcessing::thresholdValue = GetPropertyFloat(PROP_THRESHOLD);
		}
	} else if (cString::IsEqual(strName, PROP_AUTO_ANGLE)) {
		autoAngle = GetPropertyBool(PROP_AUTO_ANGLE);
		camAngle = GetPropertyFloat(PROP_ALPHA);
		recalcIPMMatrix();
	} else if (cString::IsEqual(strName, PROP_PPARKING_WIDTH)) {
		prefs.pParkingWidth = GetPropertyFloat(PROP_PPARKING_WIDTH);
	} else if (cString::IsEqual(strName, PROP_PPARKING_WIDTH_DEV)) {
		prefs.pParkingWidthDev = GetPropertyFloat(PROP_PPARKING_WIDTH_DEV);
	} else if (cString::IsEqual(strName, PROP_CROSSING_DEBUG)) {
		prefs.debugCrossings = GetPropertyBool(PROP_CROSSING_DEBUG);
	} else if (cString::IsEqual(strName, PROP_CROSSING_CORNERL1)) {
		prefs.crossingCornerLength1 = GetPropertyFloat(PROP_CROSSING_CORNERL1);
	} else if (cString::IsEqual(strName, PROP_CROSSING_CORNERL2)) {
		prefs.crossingCornerLength2 = GetPropertyFloat(PROP_CROSSING_CORNERL2);
	} else if (cString::IsEqual(strName, PROP_DEBUG_FRAMETIMES)) {
		prefs.debugTimings = GetPropertyBool(PROP_DEBUG_FRAMETIMES);
	} else if (cString::IsEqual(strName, PROP_PARKING_OFFSET)) {
		prefs.pParkingSearchOffset = GetPropertyFloat(PROP_PARKING_OFFSET);
	} else if (cString::IsEqual(strName, PROP_DEBUG_LANEFINDER)) {
		prefs.debugLaneFinder = GetPropertyBool(PROP_DEBUG_LANEFINDER);
	} else if (cString::IsEqual(strName, PROP_DETECTOR_RX)) {
		prefs.rightLaneSearchX = GetPropertyInt(PROP_DETECTOR_RX);
	} else if (cString::IsEqual(strName, PROP_DETECTOR_RY)) {
		prefs.rightLaneSearchY = GetPropertyInt(PROP_DETECTOR_RY);
	} else if (cString::IsEqual(strName, PROP_DETECTOR_LX)) {
		prefs.leftLaneSearchX = GetPropertyInt(PROP_DETECTOR_LX);
	} else if (cString::IsEqual(strName, PROP_DETECTOR_LY)) {
		prefs.leftLaneSearchY = GetPropertyInt(PROP_DETECTOR_LY);
	} else if (cString::IsEqual(strName, PROP_DETECTOR_OFFS_CRX)) {
		prefs.leftLaneSearchCurveRightOffs.x = GetPropertyInt(PROP_DETECTOR_OFFS_CRX, prefs.leftLaneSearchCurveRightOffs.x);
	} else if (cString::IsEqual(strName, PROP_DETECTOR_OFFS_CRY)) {
		prefs.leftLaneSearchCurveRightOffs.y = GetPropertyInt(PROP_DETECTOR_OFFS_CRY, prefs.leftLaneSearchCurveRightOffs.y);
	} else if (cString::IsEqual(strName, PROP_DETECTOR_OFFS_CLX)) {
		prefs.leftLaneSearchCurveLeftOffs.x = GetPropertyInt(PROP_DETECTOR_OFFS_CLX, prefs.leftLaneSearchCurveLeftOffs.x);
	} else if (cString::IsEqual(strName, PROP_DETECTOR_OFFS_CLY)) {
		prefs.leftLaneSearchCurveLeftOffs.y = GetPropertyInt(PROP_DETECTOR_OFFS_CLY, prefs.leftLaneSearchCurveLeftOffs.y);
	}
	RETURN_NOERROR;
}

tResult SR_ImageProcessing::transmitImage(cVideoPin *pin, const Mat &m)
{
	IMediaSample *pNewSample;
	RETURN_IF_FAILED(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &pNewSample));
	RETURN_IF_FAILED(pNewSample->AllocBuffer(m.elemSize() * m.total()));
	RETURN_IF_FAILED(pNewSample->Update(0, m.data, m.elemSize() * m.total(), 0));
	pin->Transmit(pNewSample);
	pNewSample->Unref();
    RETURN_NOERROR;
}

tResult SR_ImageProcessing::transmitTopDownImage(const Mat &m)
{
	MutexLocker ml(&topDownMutex);
	if (prefs.outputType == Threshold) {
		for (size_t i = 0; i < gridPoints.size(); ++i) {
			cv::circle(m, gridPoints[i], 4, CV_RGB(255, i == 0 ? 255 : 0, i == 1 ? 255 : 0), 1, CV_AA);
		}
	}
    return transmitImage(&topDownOutput, m);
}

tResult SR_ImageProcessing::transmitFilteredImage(const Mat &m)
{
	MutexLocker ml(&filteredMutex);
	return transmitImage(&filteredOutput, m);
}

tResult SR_ImageProcessing::transmitThresholdImage(const Mat &m)
{
	MutexLocker ml(&threshMutex);
	return transmitImage(&thresholdOutput, m);
}

tResult SR_ImageProcessing::transmitLaneData(const LaneData &ld)
{
	MutexLocker ml(&laneDataMutex);
	SEND_MEDIA_SAMPLE(laneDataPin, LaneData, ld, _clock->GetStreamTime());
	RETURN_NOERROR;
}

tResult SR_ImageProcessing::transmitCrossingData(const CrossingDetectionEvent &event)
{
	MutexLocker ml(&crossingDataMutex);
	SEND_MEDIA_SAMPLE(crossingPin, CrossingDetectionEvent, event, _clock->GetStreamTime());
	RETURN_NOERROR;
}

tResult SR_ImageProcessing::transmitParkingData(const ParkingSearchRequest &r)
{
	MutexLocker ml(&parkingDataMutex);
	SEND_MEDIA_SAMPLE(parkingDistPin, ParkingSearchRequest, r, _clock->GetStreamTime());
	RETURN_NOERROR;
}

void SR_ImageProcessing::setParking(bool parking)
{
	prefs.pParking = parking;
}

bool SR_ImageProcessing::shouldOutputTopDown() const
{
	return topDownConnected;
}

bool SR_ImageProcessing::shouldOutputFiltered() const
{
	return filteredConnected;
}

bool SR_ImageProcessing::shouldOutputThreshold() const
{
	return thresholdConnected;
}

void SR_ImageProcessing::applyHomography(Mat *source, Mat *target)
{
	MutexLocker ml(&ipmMutex);
	ipm.applyHomography(*source, *target);
}

SR_ImageProcessing::ImageProcPrefs SR_ImageProcessing::getPrefs() const
{
	MutexLocker ml(&prefMutex);
	return prefs;
}

tResult SR_ImageProcessing::ProcessInput(IMediaSample *pSample, tTimeStamp timeStamp)
{
	// VideoInput
	RETURN_IF_POINTER_NULL(pSample);

	// dispatch to workers
	workers[currentFrame % workers.size()]->requestProcessing(pSample, timeStamp, currentPose);

	RETURN_NOERROR;
}

double getAngle(int pixel)
{
	return (M_PI * 45.0/180.0) * pixel / 240;
}

tResult SR_ImageProcessing::ProcessDepth(IMediaSample *pSample)
{
	// VideoInput
	RETURN_IF_POINTER_NULL(pSample);

	const tVoid* l_pSrcBuffer;
	RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
	pSample->Unlock(l_pSrcBuffer);
	uint16_t *image = (uint16_t*)l_pSrcBuffer;

	// Debug stuff
	// line graph of distance along middle, 1pix = 1dm
	/*
	for (int i = 0; i < 240; ++i) {
		//cv::circle(image, Point(image.at<uint16_t>(160, i) / 10.0, i), 1, Scalar(UINT16_MAX));
		uint16_t dist = image[i * 320 + 160] / 100.0;
		if (dist > 319) {
			dist = 319;
		}
		image[i * 320 + dist] = UINT16_MAX;
	}
	*/
	// mark stuff that's 0 grey
	/*
	for (int i = 0; i < 240*320; ++i) {
		if (image[i] == 0) {
			image[i] = UINT16_MAX / 2;
		}
	}
	*/

	// Depth Camera FOV is 45 deg
	// Res is 1mm, SET THIS IN THE CONFIG FILE!
	// Image size is 320x240
	int center = 140;
	int step = 10;
	int xCenter = 160;
	double angleToGround = 0;
	int steps = 0;
	for (int i = 1; i < 5; ++i) {
		uint16_t sb = image[xCenter + (center - i * step) * 320];
		uint16_t sc = image[xCenter + (center + i * step) * 320];
		if (sb == 0 || sc == 0 || sb == UINT16_MAX || sc == UINT16_MAX) {
			continue;
		}
		double alpha = getAngle(2 * i * step);
		double sa = sqrt(sb * sb + sc * sc - 2.0 * sb * sc * cos(alpha));
		//double gamma = asin(sc * sin(alpha) / sa);
		double gamma = acos((sc * sc - sb * sb - sa * sa) / (-2.0 * sa * sb));
		angleToGround += gamma + getAngle(center - i * step - 120);
		++steps;
		//LOG_INFO(cString::Format("sa %g sb %u sc %u alpha %g gamma %g", sa, sb, sc, alpha, gamma));
	}
	if (steps > 0 && angleToGround == angleToGround) {
		angleToGround /= steps;
		camAngle = (angleToGround * 180.0 / M_PI); // degrees
		if (camAngles.size() < 16) {
			camAngles.push_back(camAngle);
			++camAngleIdx;
			camAngleIdx %= 16;
		} else {
			camAngles[camAngleIdx] = camAngle;
			camAngleIdx = (camAngleIdx + 1) % camAngles.size();
		}
		camAngle = 0;
		for (size_t i = 0; i < camAngles.size(); ++i) {
			camAngle += camAngles[i];
		}
		camAngle /= camAngles.size();
		//LOG_INFO(cString::Format("angle to ground is %g", camAngle));
		recalcIPMMatrix();
		if (prefs.debugPrintoutEnabled) {
			logger.log() << "Cam angle " << camAngle << " 1m = " << 1.0 / prefs.meterPerPixel << endl;
		}
	}
	RETURN_NOERROR;
}

void SR_ImageProcessing::recalcIPMMatrix()
{
	int width = 640;
	int height = 480;

	double a = camAngle / 180.0 * M_PI;
	double sinalpha = sin(a);
	double cosalpha = cos(a);
	// OffsetZ is height above ground
	// OffsetY is forward
	// Image plane here is x, z, so we have to exchange y and z
	Mat camR_T = (Mat_<double>(3,3) << 1, 0, 0,
				0, cosalpha, -sinalpha,
				sinalpha, cosalpha, 0);
	Mat camT = camR_T * (Mat_<double>(3,1) << camOffsetX, -camOffsetZ, camOffsetY);
	Mat srcCamMatrix = (Mat_<double>(3, 4) << 1, 0, 0, camT.at<double>(0),
						 0, cosalpha, -sinalpha, -camT.at<double>(1),
						 0, sinalpha, cosalpha, camT.at<double>(2));
	Mat destCamMatrix = (Mat_<double>(3, 4) << 1, 0, 0, 0,
						 0, 0, -1, topDownDist,
						 0, 1, 0, topDownHeight);
	vector<Mat> worldPoints;
	worldPoints.push_back((Mat_<double>(1, 4) << 1, 0, topDownDist + 1, 1));
	worldPoints.push_back((Mat_<double>(1, 4) << 1, 0, topDownDist + -1, 1));
	worldPoints.push_back((Mat_<double>(1, 4) << -1, 0, topDownDist + -1, 1));
	worldPoints.push_back((Mat_<double>(1, 4) << -1, 0, topDownDist + 1, 1));
	vector<Mat> testPoints;
	static double testGridLength = .40; // half the sidelength
	static double testGridCenterDist = 1.50; // distance to center of grid
	// top right
	testPoints.push_back((Mat_<double>(1, 4) <<  testGridLength, 0, testGridCenterDist + testGridLength, 1));
	// bottom right
	testPoints.push_back((Mat_<double>(1, 4) <<  testGridLength, 0, testGridCenterDist - testGridLength, 1));
	// bottom left
	testPoints.push_back((Mat_<double>(1, 4) << -testGridLength, 0, testGridCenterDist - testGridLength, 1));
	// top left
	testPoints.push_back((Mat_<double>(1, 4) << -testGridLength, 0, testGridCenterDist + testGridLength, 1));

	origPoints.clear();
	dstPoints.clear();
	gridPoints.clear();

	std::string s;
	std::stringstream ss(s);

	ss << "Transformation mappings:" << endl;

	for (size_t i = 0; i < worldPoints.size(); ++i) {
		Mat p1 = m_Intrinsics * (srcCamMatrix * worldPoints.at(i).t());
		origPoints.push_back(Point2f(p1.at<double>(0,0) / p1.at<double>(0,2), p1.at<double>(0,1) / p1.at<double>(0,2)));
		p1 = m_IntrinsicsTopDown * (destCamMatrix * worldPoints.at(i).t());
		dstPoints.push_back(Point2f(p1.at<double>(0,0) / p1.at<double>(0,2), p1.at<double>(0,1) / p1.at<double>(0,2)));
		p1 = m_IntrinsicsTopDown * (destCamMatrix * testPoints.at(i).t());
		gridPoints.push_back(Point2f(p1.at<double>(0,0) / p1.at<double>(0,2), p1.at<double>(0,1) / p1.at<double>(0,2)));
		ss << "point " << i << ": " << origPoints.at(i) << " -> " << dstPoints.at(i) << endl;
	}

	Point2f topPoint = dstPoints[0];
	Point2f botPoint = dstPoints[1];
	// orig points are 2 meters apart
	prefs.meterPerPixel = 2.0 / (botPoint.y - topPoint.y);
	prefs.meterToTopDownCenter = topDownDist;

	ss << endl << "Source cam matrix:" << endl << srcCamMatrix << endl
	   << "Dest cam matrix:" << endl << destCamMatrix << endl;
	ss << "Meter Per Pixel = " << prefs.meterPerPixel << endl;

	MutexLocker ml(&ipmMutex);
	ipm = IPM(Size(width, height), Size(width, height), origPoints, dstPoints);
	Point2i top = ipm.applyHomographyInv(Point(320, 0));
	prefs.validImageRegion.clear();
	prefs.validImageRegion.push_back(ipm.applyHomography(Point(0, top.y)));
	prefs.validImageRegion.push_back(ipm.applyHomography(Point(0, 480)));
	prefs.validImageRegion.push_back(ipm.applyHomography(Point(640, 480)));
	prefs.validImageRegion.push_back(ipm.applyHomography(Point(640, top.y)));
	ss << endl << "ImageRegion" << endl << prefs.validImageRegion[0] << endl << prefs.validImageRegion[1] << endl << prefs.validImageRegion[2] << endl << prefs.validImageRegion[3] << endl;
	logger.log() << ss.str();
}
