#include "PathPlanner.h"
#include "../misc/CrossingHelpers.h"
#include "../misc/OdometryHelpers.h"
#include "../misc/LineHelpers.h"

#include <adtf_base.h>
#include <maneuverdata.h>
#include <opencv/cv.h>
#include <opencv2/imgproc.hpp>

#define MILLISECONDS * 1000
#define SECONDS *1000000

const int debugimgsize = 1024;
const int debugimgcenter = debugimgsize / 2;

using namespace cv;

ADTF_FILTER_PLUGIN("SpaceRacer Path Planner", OID_SR_PATHPLANNER, PathPlanner)

#define PROP_DEBUG_DP "Debug::Driving Path"
#define PROP_LF_POSE_DIST "LaneFollower::Command distance"
#define PROP_LF_UPDATE_DIST "LaneFollower::Send new pose after"
#define PROP_LF_SPEED	"LaneFollower::Speed"
#define PROP_PARKING_SCANSPEED "Parking::Scan driving speed"
#define PROP_PARKING_MIN_SIGN_HITS "Parking::Min sign detections"
#define PROP_CROSSINGS_MIN_DETECTIONS "Crossings::Min detections"
#define PROP_CROSSINGS_MAX_DIST "Crossings::Max dist"
#define PROP_CROSSINGS_CHECK_OBST "Crossings::Check for traffic"
#define PROP_CROSSINGS_TURN_VEL "Crossings::Turning speed"
#define PROP_CROSSINGS_POSE_OFFS_FL "Crossings::Pose offset fwd left"
#define PROP_CROSSINGS_POSE_OFFS_FR "Crossings::Pose offset fwd right"
#define PROP_CROSSINGS_POSE_OFFS_CL "Crossings::Pose offset cross left"
#define PROP_CROSSINGS_POSE_OFFS_CR "Crossings::Pose offset cross right"
#define PROP_CROSSINGS_STOP_OFFS "Crossings::Pose offset stop"
#define PROP_CROSSINGS_STOP_LEFT_OFFS "Crossings::Intermediate pose for left turn"
#define PROP_DEBUG_SCALE "Debug::Drawing scale [pix/m]"
#define PROP_DEBUG_PERSISTENCY "Debug::Drawing persistency"
#define PROP_DEBUG_POSES "Debug::Pose Requests"

#define PROP_P_NUM "Path::Number of initial points"
#define PROP_P_STEP "Path::Step size"
#define PROP_P_REJ_DIST "Path::Rejection distance"
#define PROP_P_W_INC "Path::Weight increment"
#define PROP_P_W_INITIAL "Path::Initial weight"
#define PROP_P_W_AGEBASED "Path::Use age based weighting"
#define PROP_P_INTERP_POSE "Path::Use interpolated goal poses"

#define PROP_P_W_U_E "Path::Weights::Update::Extrapolated Point"
#define PROP_P_W_U_R "Path::Weights::Update::Right Lane Point"
#define PROP_P_W_U_L "Path::Weights::Update::Left Lane Point"
#define PROP_P_W_U_C "Path::Weights::Update::Center Lane Point"
#define PROP_P_W_E_E "Path::Weights::Extrapolate::Extrapolated Point"
#define PROP_P_W_E_R "Path::Weights::Extrapolate::Right Lane Point"
#define PROP_P_W_E_L "Path::Weights::Extrapolate::Left Lane Point"
#define PROP_P_W_E_C "Path::Weights::Extrapolate::Center Lane Point"

#define PROP_CROSSINGS_MIN_FREE_TICKS "Crossings::Min Free Ticks"

PathPlanner::PathPlanner(const char *name) : cFilter(name), debugImg(debugimgsize, debugimgsize, CV_8UC3, Scalar(0, 0, 0, 0))
{
	logger = NULL;
	debugDrivingPath = false;
	SetPropertyBool(PROP_DEBUG_DP, debugDrivingPath);
	SetPropertyBool(PROP_DEBUG_DP NSSUBPROP_ISCHANGEABLE, tTrue);
	lfCommandDist = .7;
	SetPropertyFloat(PROP_LF_POSE_DIST, lfCommandDist);
	lfUpdateDist = .5;
	SetPropertyFloat(PROP_LF_UPDATE_DIST, lfUpdateDist);
	lfSpeed = .7;
	SetPropertyFloat(PROP_LF_SPEED, lfSpeed);
	parkScanSpeed = .3;
	SetPropertyFloat(PROP_PARKING_SCANSPEED, parkScanSpeed);
	minCrossingDetections = 5;
	SetPropertyInt(PROP_CROSSINGS_MIN_DETECTIONS, minCrossingDetections);
	SetPropertyInt(PROP_CROSSINGS_MIN_DETECTIONS NSSUBPROP_MIN, 1);
	maxCrossingDist = 2;
	SetPropertyFloat(PROP_CROSSINGS_MAX_DIST, maxCrossingDist);
	debugDrawScale = 64;
	SetPropertyInt(PROP_DEBUG_SCALE, debugDrawScale);
	SetPropertyInt(PROP_DEBUG_SCALE NSSUBPROP_MIN, 1);
	debugDrawPersistency = .99;
	SetPropertyFloat(PROP_DEBUG_PERSISTENCY, debugDrawPersistency);
	SetPropertyFloat(PROP_DEBUG_PERSISTENCY NSSUBPROP_MIN, 0);
	SetPropertyFloat(PROP_DEBUG_PERSISTENCY NSSUBPROP_MAX, 1);
	SetPropertyBool(PROP_DEBUG_PERSISTENCY NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(PROP_P_NUM, path.getNumPoints());
	SetPropertyFloat(PROP_P_STEP, path.getStepSize());
	SetPropertyFloat(PROP_P_REJ_DIST, path.getRejectionDistance());
	SetPropertyFloat(PROP_P_W_INC, path.getWeightIncrement());
	SetPropertyFloat(PROP_P_W_INITIAL, path.getInitialPointWeight());
	SetPropertyBool(PROP_P_W_AGEBASED, path.getAgeBasedWeighting());
	useInterpCommand = true;
	SetPropertyBool(PROP_P_INTERP_POSE, useInterpCommand);
	crossingTurnVel = lfSpeed;
	SetPropertyFloat(PROP_CROSSINGS_TURN_VEL, crossingTurnVel);
	checkObstacles = true;
	SetPropertyBool(PROP_CROSSINGS_CHECK_OBST, checkObstacles);
	crossingPoseOffsetFwdLeft = .2;
	crossingPoseOffsetFwdRight = .5;
	crossingPoseOffsetCrossLeft = 0;
	crossingPoseOffsetCrossRight = 0;
	crossingStopOffset = 0;
	crossingStopLeftIntermediateOffs = 0;
	SetPropertyFloat(PROP_CROSSINGS_POSE_OFFS_FL, crossingPoseOffsetFwdLeft);
	SetPropertyFloat(PROP_CROSSINGS_POSE_OFFS_FR, crossingPoseOffsetFwdRight);
	SetPropertyFloat(PROP_CROSSINGS_POSE_OFFS_CL, crossingPoseOffsetCrossLeft);
	SetPropertyFloat(PROP_CROSSINGS_POSE_OFFS_CR, crossingPoseOffsetCrossRight);
	SetPropertyFloat(PROP_CROSSINGS_STOP_OFFS, crossingStopOffset);
	SetPropertyFloat(PROP_CROSSINGS_STOP_LEFT_OFFS, crossingStopLeftIntermediateOffs);
	SetPropertyBool(PROP_CROSSINGS_POSE_OFFS_FL NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_CROSSINGS_POSE_OFFS_FR NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_CROSSINGS_POSE_OFFS_CL NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_CROSSINGS_POSE_OFFS_CR NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_CROSSINGS_STOP_OFFS NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_CROSSINGS_STOP_LEFT_OFFS NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(PROP_CROSSINGS_STOP_OFFS NSSUBPROP_DESCRIPTION, "Positive values mean further away from crossing center");
	SetPropertyStr(PROP_CROSSINGS_POSE_OFFS_CL NSSUBPROP_DESCRIPTION, "Positive values mean further away from middle lane towards outer lane");
	SetPropertyStr(PROP_CROSSINGS_POSE_OFFS_CR NSSUBPROP_DESCRIPTION, "Positive values mean further away from middle lane towards outer lane");
	SetPropertyStr(PROP_CROSSINGS_POSE_OFFS_FL NSSUBPROP_DESCRIPTION, "Positive values mean further away from crossing center");
	SetPropertyStr(PROP_CROSSINGS_POSE_OFFS_FR NSSUBPROP_DESCRIPTION, "Positive values mean further away from crossing center");
	minParkingSignHits = 3;
	SetPropertyInt(PROP_PARKING_MIN_SIGN_HITS, minParkingSignHits);
	debugPoseRequests = false;
	SetPropertyBool(PROP_DEBUG_POSES, debugPoseRequests);
	SetPropertyBool(PROP_DEBUG_POSES NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat(PROP_P_W_U_E, path.getWeightUpdExtr());
	SetPropertyFloat(PROP_P_W_U_R, path.getWeightUpdRight());
	SetPropertyFloat(PROP_P_W_U_L, path.getWeightUpdLeft());
	SetPropertyFloat(PROP_P_W_U_C, path.getWeightUpdCenter());

	SetPropertyFloat(PROP_P_W_E_E, path.getWeightExtrExtr());
	SetPropertyFloat(PROP_P_W_E_R, path.getWeightExtrRight());
	SetPropertyFloat(PROP_P_W_E_L, path.getWeightExtrLeft());
	SetPropertyFloat(PROP_P_W_E_C, path.getWeightExtrCenter());

	SetPropertyBool(PROP_P_W_U_E NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_P_W_U_R NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_P_W_U_L NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_P_W_U_C NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyBool(PROP_P_W_E_E NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_P_W_E_R NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_P_W_E_L NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool(PROP_P_W_E_C NSSUBPROP_ISCHANGEABLE, tTrue);

	minFreeTicks = 30;
	SetPropertyInt(PROP_CROSSINGS_MIN_FREE_TICKS, minFreeTicks);

	shouldLaneFollow = false;
	currentManeuver = ManeuverIdle;
	blinkerStatus = BlinkerOff;
	state = Ready;
	freeCounter = 0;
}

PathPlanner::~PathPlanner()
{

}

tResult PathPlanner::PropertyChanged(const tChar *strName)
{
	if (cString::IsEqual(strName, PROP_DEBUG_DP)) {
		debugDrivingPath = GetPropertyBool(PROP_DEBUG_DP);
	} else if (cString::IsEqual(strName, PROP_DEBUG_PERSISTENCY)) {
		debugDrawPersistency = GetPropertyFloat(PROP_DEBUG_PERSISTENCY);
	} else if (cString::IsEqual(strName, PROP_CROSSINGS_POSE_OFFS_FL)) {
		crossingPoseOffsetFwdLeft = GetPropertyFloat(PROP_CROSSINGS_POSE_OFFS_FL);
	} else if (cString::IsEqual(strName, PROP_CROSSINGS_POSE_OFFS_FR)) {
		crossingPoseOffsetFwdRight = GetPropertyFloat(PROP_CROSSINGS_POSE_OFFS_FR);
	} else if (cString::IsEqual(strName, PROP_CROSSINGS_POSE_OFFS_CL)) {
		crossingPoseOffsetCrossLeft = GetPropertyFloat(PROP_CROSSINGS_POSE_OFFS_CL);
	} else if (cString::IsEqual(strName, PROP_CROSSINGS_POSE_OFFS_CR)) {
		crossingPoseOffsetCrossRight = GetPropertyFloat(PROP_CROSSINGS_POSE_OFFS_CR);
	} else if (cString::IsEqual(strName, PROP_CROSSINGS_STOP_OFFS)) {
		crossingStopOffset = GetPropertyFloat(PROP_CROSSINGS_STOP_OFFS);
	} else if (cString::IsEqual(strName, PROP_CROSSINGS_STOP_LEFT_OFFS)) {
		crossingStopLeftIntermediateOffs = GetPropertyFloat(PROP_CROSSINGS_STOP_LEFT_OFFS);
	} else if (cString::IsEqual(strName, PROP_DEBUG_POSES)) {
		debugPoseRequests = GetPropertyBool(PROP_DEBUG_POSES);
	} else if (cString::IsEqual(strName, PROP_P_W_U_E)) {
		path.setWeightUpdExtr(GetPropertyFloat(PROP_P_W_U_E));
	} else if (cString::IsEqual(strName, PROP_P_W_U_R)) {
		path.setWeightUpdRight(GetPropertyFloat(PROP_P_W_U_R));
	} else if (cString::IsEqual(strName, PROP_P_W_U_L)) {
		path.setWeightUpdLeft(GetPropertyFloat(PROP_P_W_U_L));
	} else if (cString::IsEqual(strName, PROP_P_W_U_C)) {
		path.setWeightUpdCenter(GetPropertyFloat(PROP_P_W_U_C));
	} else if (cString::IsEqual(strName, PROP_P_W_E_E)) {
		path.setWeightExtrExtr(GetPropertyFloat(PROP_P_W_E_E));
	} else if (cString::IsEqual(strName, PROP_P_W_E_R)) {
		path.setWeightExtrRight(GetPropertyFloat(PROP_P_W_E_R));
	} else if (cString::IsEqual(strName, PROP_P_W_E_L)) {
		path.setWeightExtrLeft(GetPropertyFloat(PROP_P_W_E_L));
	} else if (cString::IsEqual(strName, PROP_P_W_E_C)) {
		path.setWeightExtrCenter(GetPropertyFloat(PROP_P_W_E_C));
	}
	RETURN_NOERROR;
}

tResult PathPlanner::Init(adtf::cFilter::tInitStage stage, IException **__exception_ptr)
{
	RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr));

	if (stage == StageFirst) {
		REGISTER_MEDIA_PIN(posePin, MEDIA_TYPE_ODOMETRY, MEDIA_SUBTYPE_ODOMETRY_POSE, "Pose");
		REGISTER_MEDIA_PIN(crossingPin, MEDIA_TYPE_MAPDATA, MEDIA_SUBTYPE_MAPDATA_CROSSING_EVENT, "Crossings");
		REGISTER_MEDIA_PIN(lanePin, MEDIA_TYPE_MAPDATA, MEDIA_SUBTYPE_MAPDATA_LANEDATA, "Lanes");
		REGISTER_MEDIA_PIN(parkingReferencePoint, MEDIA_TYPE_ODOMETRY, MEDIA_SUBTYPE_ODOMETRY_POSE, "ParkingReferencePoint");
		REGISTER_MEDIA_PIN(blockedSpotsPin, MEDIA_TYPE_MAPDATA, MEDIA_SUBTYPE_MAPDATA_BLOCKEDSPOTS, "BlockedSpots");
		REGISTER_MEDIA_PIN(signPin, MEDIA_TYPE_MAPDATA, MEDIA_SUBTYPE_MAPDATA_SIGN, "Signs");
		REGISTER_MEDIA_PIN(maneuverPin, MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_UINT8, "Maneuver");
		INIT_WRAPPED_PIN(keyboardCommandPin, "KeyboardCommand");
		INIT_WRAPPED_PIN(controllerPoseReachedPin, "PoseReached");
		INIT_WRAPPED_PIN(parkingBlinker, "ParkingBlinkerStart");
		REGISTER_MEDIA_PIN(initialStatePin, MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_UINT8, "Initial_State");

		REGISTER_MEDIA_PIN(controllerOutputPin, MEDIA_TYPE_CONTROLLERCOMMAND, MEDIA_SUBTYPE_CONTROLLER_COMMAND, "Command");
		REGISTER_MEDIA_PIN(maneuverResponsePin, MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_UINT8, "Maneuver_Response");
		INIT_WRAPPED_PIN(parkingSearchRequestPin, "ParkingSearchRequest");
		REGISTER_MEDIA_PIN(resetLaneTrackingPin, MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_UINT8, "Reset_Lane_Tracking");
		INIT_WRAPPED_PIN(blinkerLeftPin, "BlinkerLeft");
		INIT_WRAPPED_PIN(blinkerRightPin, "BlinkerRight");
		INIT_WRAPPED_PIN(blinkerHazardPin, "BlinkerHazard");
		RETURN_IF_FAILED(videoOut.Create("Debug_Video", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&videoOut));
		REGISTER_MEDIA_PIN(requestInitialStatePin, MEDIA_TYPE_USER, 0, "Request_Initial_State");
		outFormat.nPixelFormat = IImage::PF_RGB_888;
		outFormat.nWidth = debugimgsize;
		outFormat.nHeight = debugimgsize;
		outFormat.nBitsPerPixel = 24;
		outFormat.nBytesPerLine = debugimgsize * 3;
		outFormat.nSize = debugimgsize * debugimgsize * 3;
		outFormat.nPaletteSize = 0;
		videoOut.SetFormat(&outFormat, NULL);
	} else if (stage == StageNormal) {
		delete logger;
		logger = new SR_Logger(OID_SR_PATHPLANNER);
		logger->GLogTrap("========== PathPlanner starting up ==========");
		debugDrivingPath = GetPropertyBool(PROP_DEBUG_DP, debugDrivingPath);
		lfCommandDist = GetPropertyFloat(PROP_LF_POSE_DIST, lfCommandDist);
		lfUpdateDist = GetPropertyFloat(PROP_LF_UPDATE_DIST, lfUpdateDist);
		lfSpeed = GetPropertyFloat(PROP_LF_SPEED, lfSpeed);
		parkScanSpeed = GetPropertyFloat(PROP_PARKING_SCANSPEED, parkScanSpeed);
		minCrossingDetections = GetPropertyInt(PROP_CROSSINGS_MIN_DETECTIONS, minCrossingDetections);
		maxCrossingDist = GetPropertyFloat(PROP_CROSSINGS_MAX_DIST, maxCrossingDist);
		debugDrawScale = GetPropertyFloat(PROP_DEBUG_SCALE, debugDrawScale);
		debugDrawPersistency = GetPropertyFloat(PROP_DEBUG_PERSISTENCY, debugDrawPersistency);
		path.setStepSize(GetPropertyFloat(PROP_P_STEP, path.getStepSize()));
		path.setRejectionDistance(GetPropertyFloat(PROP_P_REJ_DIST, path.getRejectionDistance()));
		path.setWeightIncrement(GetPropertyFloat(PROP_P_W_INC, path.getWeightIncrement()));
		path.setInitialPointWeight(GetPropertyFloat(PROP_P_W_INITIAL, path.getInitialPointWeight()));
		path.setAgeBasedWeighting(GetPropertyBool(PROP_P_W_AGEBASED, path.getAgeBasedWeighting()));
		path.setNumPoints(GetPropertyInt(PROP_P_NUM, path.getNumPoints()));
		useInterpCommand = GetPropertyBool(PROP_P_INTERP_POSE, useInterpCommand);
		checkObstacles = GetPropertyBool(PROP_CROSSINGS_CHECK_OBST, checkObstacles);
		crossingTurnVel = GetPropertyFloat(PROP_CROSSINGS_TURN_VEL, crossingTurnVel);
		crossingPoseOffsetFwdLeft = GetPropertyFloat(PROP_CROSSINGS_POSE_OFFS_FL, crossingPoseOffsetFwdLeft);
		crossingPoseOffsetFwdRight = GetPropertyFloat(PROP_CROSSINGS_POSE_OFFS_FR, crossingPoseOffsetFwdRight);
		crossingPoseOffsetCrossLeft = GetPropertyFloat(PROP_CROSSINGS_POSE_OFFS_CL, crossingPoseOffsetCrossLeft);
		crossingPoseOffsetCrossRight = GetPropertyFloat(PROP_CROSSINGS_POSE_OFFS_CR, crossingPoseOffsetCrossRight);
		crossingStopOffset = GetPropertyFloat(PROP_CROSSINGS_STOP_OFFS, crossingStopOffset);
		crossingStopLeftIntermediateOffs = GetPropertyFloat(PROP_CROSSINGS_STOP_LEFT_OFFS, crossingStopLeftIntermediateOffs);
		minParkingSignHits = GetPropertyInt(PROP_PARKING_MIN_SIGN_HITS, minParkingSignHits);
		debugPoseRequests = GetPropertyBool(PROP_DEBUG_POSES, debugPoseRequests);

		path.setWeightUpdExtr(GetPropertyFloat(PROP_P_W_U_E, path.getWeightUpdExtr()));
		path.setWeightUpdRight(GetPropertyFloat(PROP_P_W_U_R, path.getWeightUpdRight()));
		path.setWeightUpdLeft(GetPropertyFloat(PROP_P_W_U_L, path.getWeightUpdLeft()));
		path.setWeightUpdCenter(GetPropertyFloat(PROP_P_W_U_C, path.getWeightUpdCenter()));

		path.setWeightExtrExtr(GetPropertyFloat(PROP_P_W_E_E, path.getWeightExtrExtr()));
		path.setWeightExtrRight(GetPropertyFloat(PROP_P_W_E_R, path.getWeightExtrRight()));
		path.setWeightExtrLeft(GetPropertyFloat(PROP_P_W_E_L, path.getWeightExtrLeft()));
		path.setWeightExtrCenter(GetPropertyFloat(PROP_P_W_E_C, path.getWeightExtrCenter()));

		minFreeTicks = GetPropertyInt(PROP_CROSSINGS_MIN_FREE_TICKS, minFreeTicks);

		shouldLaneFollow = false;
		poseRequested = false;
		currentManeuver = ManeuverIdle;
		timerRunning = false;
		state = Ready;
		lastCommandPos = Point2d(0, 0);
		freeCounter = 0;
	} else if (stage == StageGraphReady) {
		debugImg.setTo(Scalar(0, 0, 0, 0));
		currBlockedSpots.straight_lane = false;
		currBlockedSpots.right_lane = false;
		freeCounter = 0;
	}
	RETURN_NOERROR;
}

tResult PathPlanner::Shutdown(adtf::cFilter::tInitStage stage, IException **__exception_ptr)
{
	RETURN_IF_FAILED(cFilter::Shutdown(stage, __exception_ptr));
	delete logger;
	logger = NULL;
	RETURN_NOERROR;
}

tResult PathPlanner::OnPinEvent(adtf::IPin *source, tInt eventCode, tInt param1, tInt param2, adtf::IMediaSample *mediaSample)
{
	if (eventCode == IPinEventSink::PE_MediaSampleReceived) {
		RETURN_IF_POINTER_NULL(mediaSample);
		if (source == &posePin) {
			READ_MEDIA_PIN(mediaSample, OdometryPose, currPose);
			processNewPose();
		} else if (source == &crossingPin) {
			CrossingDetectionEvent e;
			READ_MEDIA_PIN(mediaSample, CrossingDetectionEvent, e);
			processCrossingData(e);
		} else if (source == &lanePin) {
			READ_MEDIA_PIN(mediaSample, LaneData, lastLaneData);
			//logger->log() << "Received LaneData: " << ld.rightLane.size() << " points of right lane, " << ld.leftLane.size() << " left lane, " << ld.centerLane.size() << " center lane " << endl;
			if (shouldLaneFollow) {
				processLaneData(lastLaneData);
			} else {
				resetLaneDetector();
			}
			if (videoOut.IsConnected()) {
				if (!shouldLaneFollow) {
					processLaneData(lastLaneData);
				}
				tResult ret = debugDrawEverything(lastLaneData);
				if (ret) {
					logger->log() << "Ret was " << ret << endl;
				}
			}
		} else if (source == &parkingReferencePoint) {
			OdometryPose refPoint;
			READ_MEDIA_PIN(mediaSample, OdometryPose, refPoint);
			if (isParking() && state == WaitForParkingSpot) {
				doParking(refPoint);
			} else {
				logger->GLogTrap("Ignoring parking reference point because we aren't parking");
			}
        } else if (source == &blockedSpotsPin) {
			READ_MEDIA_PIN(mediaSample, BlockedSpots, currBlockedSpots);
			processObstacles();
		} else if (source == &signPin) {
			SignStruct s;
			READ_MEDIA_PIN(mediaSample, SignStruct, s);
			processSigns(s);
		} else if (source == &maneuverPin) {
			uint8_t val = static_cast<uint8_t>(ManeuverIdle);
			READ_MEDIA_PIN(mediaSample, uint8_t, val);
			setCurrentManeuver(static_cast<ManeuverType>(val));
		} else if (source == keyboardCommandPin.ppin) {
			float val;
			keyboardCommandPin.getValue(mediaSample, &val);
			handleKeyboardCommand(val);
		} else if (source == controllerPoseReachedPin.ppin) {
			handlePoseReached();
		} else if (source == parkingBlinker.ppin) {
			if (isParking() && state == WaitForParkingSpotScanStart) {
				setBlinker(BlinkerRight);
				logger->GLogTrap("Starting ultrasonic search");
				state = WaitForParkingSpot;
				// Send a new desired pose with reduced speed
				requestLaneTrackingPose();
			} else {
				logger->GLogTrap("Ignoring ultrasonic start");
			}
		} else if (source == &initialStatePin) {
			uint8_t val = 0;
			READ_MEDIA_PIN(mediaSample, uint8_t, val);
			handleInitialState(static_cast<InitialCarConfiguration>(val));
		}
		updateTimer();
	}
	RETURN_NOERROR;
}

tResult PathPlanner::Start(IException **__exception_ptr)
{
	RETURN_IF_FAILED(cFilter::Start(__exception_ptr));
	RETURN_NOERROR;
}

tResult PathPlanner::doParking(const OdometryPose &p)
{
	disableLaneTracking();
	logger->log() << "Parking reference pose received: " << p << " Executing maneuver" << endl;
	state = ExecutingParkingManeuver;
	ControllerCommand desiredPose;
	desiredPose.pose=p;
	desiredPose.pose.speed = 0; // Unused
	desiredPose.cruiseVelocity = 0; // Unused
	desiredPose.type= activeParking ? ParkingParallel : ParkingCross;
	return sendDesiredPose(desiredPose);
}

tResult PathPlanner::requestParkingSearch(bool parallel)
{
	activeParking=parallel;
	bool type = currentManeuver == ManeuverParallelParking;
	SEND_WRAPPED_SAMPLE(parkingSearchRequestPin, type, _clock->GetStreamTime());
	RETURN_NOERROR;
}

tResult PathPlanner::handlePoseReached()
{
	if (state == ExecutingParkingManeuver) {
		state = ParkingWaitPeriod;
		setBlinker(BlinkerHazard);
		waitStart = _clock->GetStreamTime();
		waitPeriod = 3 SECONDS;
		timerRunning = true;
	} else if (isPullout()) {
		setBlinker(BlinkerOff);
		state = Ready;
		setManeuverResponse(ManeuverResponseComplete);
	} else if (state == WaitForCrossingStop) {
		if (crossingHaveway) {
			if (currentManeuver == ManeuverLeft) {
				sendCrossingLeftIntermediateCommand();
			} else {
				logger->GLogTrap("Reached crossing, have right of way, executing maneuver");
				state = WaitForCrossingManeuverEnd;
				sendDesiredPose(crossingExitCommand);
			}
		} else {
			logger->GLogTrap("Reached crossing stop, checking for traffic");
			freeCounter = 0;
			int minFreeTicks;
			state = WaitForCrossingTraffic;
		}
	} else if (state == WaitForCrossingManeuverEnd) {
		logger->GLogTrap("Crossing maneuver complete!");
		setBlinker(BlinkerOff);
		state = Ready;
		// path will be reset when lanetracking gets enabled
		crossings.clear();
		streetSigns.clear();
		setManeuverResponse(ManeuverResponseComplete);
	} else if (state == WaitForCrossingLeftIntermediate) {
		logger->GLogTrap("Reached crossing, have right of way, executing maneuver");
		state = WaitForCrossingManeuverEnd;
		sendDesiredPose(crossingExitCommand);
	}
	if (shouldLaneFollow) {
		poseRequested = true;
		if (debugDrivingPath) {
			logger->GLogTrap("Generating new lanefollowing pose because last pose was reached.");
		}
	}
	RETURN_NOERROR;
}

void PathPlanner::handleKeyboardCommand(float command)
{
	logger->log() << "Received keyboard command " << command << endl;
	if (command == 1.0f) {
		setCurrentManeuver(ManeuverStraight);
	} else if (command == 2.0f) {
		setCurrentManeuver(ManeuverRight);
	} else if (command == 3.0f) {
		setCurrentManeuver(ManeuverLeft);
	} else if (command == 5.0f) {
		enableLaneTracking();
	} else if (command == 6.0f) {
		disableLaneTracking();
		// Attempt stopping at current pose
		ControllerCommand p;
		p.pose = currPose;
		p.type = DesiredPose;
		p.pose.speed = 0;
		p.cruiseVelocity = 0;
		sendDesiredPose(p);
	} else if (command == 7.0f) {
		setCurrentManeuver(ManeuverParallelParking);
	} else if (command == 8.0f) {
		setCurrentManeuver(ManeuverCrossParking);
	} else if (command == 9.0f) {
		if (isParking())
		{
			logger->GLogTrap("Executing regular pull out right from keyboard");
			setCurrentManeuver(ManeuverPulloutRight);
		}
		else
		{
			logger->log() << "Forcing parallel pull out from keyboard, state was " << currentManeuver << endl;
			ControllerCommand p;
			p.pose = currPose;
			p.type=PullOutParallel;
			p.pose.speed = 0;// Unused
			p.cruiseVelocity = 0; // Unused
			sendDesiredPose(p);
		}
	} else if (command == 10.0f) {
		setCurrentManeuver(ManeuverPulloutLeft);
	}
}

void PathPlanner::handleInitialState(InitialCarConfiguration c)
{
	if (c == InitialConfigurationParkedParallel) {
		currentManeuver = ManeuverParallelParking;
	} else if (c == InitialConfigurationParkedCross) {
		currentManeuver = ManeuverCrossParking;
	}
	if (state == WaitingForInitialState) {
		// We now have the initial state, try again
		state = Ready;
		setCurrentManeuver(ManeuverPulloutRight);
	}
	logger->log() << "Set initial state to " << currentManeuver << endl;
}

tResult PathPlanner::sendDesiredPose(const Point2d &pos, const Point2d dir, double speed, float cruiseSpeed)
{
	ControllerCommand nextPose;
	nextPose.pose.x = pos.x;
	nextPose.pose.y = pos.y;
	nextPose.pose.theta = 180.0 * atan2(dir.y, dir.x) / M_PI;
	nextPose.pose.speed = speed;
	nextPose.cruiseVelocity = cruiseSpeed;
	nextPose.type=DesiredPose;
	return sendDesiredPose(nextPose);
}

tResult PathPlanner::sendDesiredPose(const ControllerCommand &p)
{
	if (debugPoseRequests) {
		logger->log() << "Send desired pose: " << p << " current maneuver " << currentManeuver << " current state " << state << endl;
	}
	SEND_MEDIA_SAMPLE(controllerOutputPin, ControllerCommand, p, _clock->GetStreamTime());
	RETURN_NOERROR;
}

tResult PathPlanner::processNewPose()
{
	currHeading = getOdometryDirection(currPose);
	if (shouldLaneFollow) {
		if (norm(getOdometryPos(currPose) - lastCommandPos) > lfUpdateDist) {
			if (debugDrivingPath) {
				logger->GLogTrap("Generating new lanefollowing pose because distance to old pose was reached.");
			}
			poseRequested = true;
		}
	}
	checkForCrossing();
	RETURN_NOERROR;
}

tResult PathPlanner::processLaneData(LaneData &data)
{
	path.processLaneData(data, currPose, videoOut.IsConnected() ? this : NULL, debugDrivingPath ? logger : NULL);
	if (poseRequested) {
		poseRequested = false;
		ControllerCommand c;
		if (useInterpCommand) {
			c = path.getNextCommandInterp(currPose, lfCommandDist, debugDrivingPath ? logger : NULL);
		} else {
			c = path.getNextCommand(currPose, lfCommandDist, debugDrivingPath ? logger : NULL);
		}
		if (videoOut.IsConnected()) {
			drawArrowedLine(getOdometryPos(c.pose), getOdometryDirection(c.pose) * 20, CV_RGB(255, 0, 255));
		}
		c.pose.speed = lfSpeed;
		c.cruiseVelocity = lfSpeed;
		if (state == WaitForParkingSpot) {
			// Slow down for scan
			c.pose.speed = parkScanSpeed;
			c.cruiseVelocity = parkScanSpeed;
		}
		if (debugDrivingPath) {
			logger->log() << "Lane following: sending command " << c << endl;
		}
		lastCommandPos = Point2d(c.pose.x, c.pose.y);
		return sendDesiredPose(c);
	}
	RETURN_NOERROR;
}

void PathPlanner::processCrossingData(CrossingDetectionEvent &data)
{
	// Fuse crossing data
	mergeCrossings(crossings, data, .1);
}

tResult PathPlanner::processObstacles()
{
	if (isCrossingManeuver() && state == WaitForCrossingTraffic) {
		if (crossingDirsToCheck.r && rightIsBlocked()) {
			freeCounter = 0;
			logger->GLogTrap("Traffic from right, waiting");
			RETURN_NOERROR;
		}
		if (crossingDirsToCheck.f && straightIsBlocked()) {
			freeCounter = 0;
			logger->GLogTrap("Traffic from ahead, waiting");
			RETURN_NOERROR;
		}
		if (crossingDirsToCheck.l && leftIsBlocked()) {
			freeCounter = 0;
			logger->GLogTrap("Traffic from left, waiting");
			RETURN_NOERROR;
		}
		++freeCounter;
		if (freeCounter > minFreeTicks) {
			logger->GLogTrap("Crossing clear, executing maneuver");
			if (currentManeuver == ManeuverLeft) {
				sendCrossingLeftIntermediateCommand();
			} else {
				state = WaitForCrossingManeuverEnd;
				sendDesiredPose(crossingExitCommand);
			}
		}
	}
	RETURN_NOERROR;
}

tResult PathPlanner::processSigns(SignStruct &data)
{
	int hits = mergeSign(data);
	if (state == WaitForParkingSign && data.id == RoadSignParkingarea && hits >= minParkingSignHits) {
		logger->GLogTrap("Parking sign found. Starting parking line search");
		state = WaitForParkingSpotScanStart;
		// start search from here
		requestParkingSearch(currentManeuver == ManeuverParallelParking);
	}
	RETURN_NOERROR;
}

tResult PathPlanner::setBlinker(PathPlanner::BlinkerValue value)
{
	if (blinkerStatus == value) {
		RETURN_NOERROR;
	}
	bool val = true;
	if (value == BlinkerLeft) {
		SEND_WRAPPED_SAMPLE(blinkerLeftPin, val, _clock->GetStreamTime());
	} else if (value == BlinkerRight) {
		SEND_WRAPPED_SAMPLE(blinkerRightPin, val, _clock->GetStreamTime());
	} else if (value == BlinkerHazard) {
		SEND_WRAPPED_SAMPLE(blinkerHazardPin, val, _clock->GetStreamTime());
	} else if (value == BlinkerOff) {
		val = false;
		if (blinkerStatus == BlinkerLeft) {
			SEND_WRAPPED_SAMPLE(blinkerLeftPin, val, _clock->GetStreamTime());
		} else if (blinkerStatus == BlinkerRight) {
			SEND_WRAPPED_SAMPLE(blinkerRightPin, val, _clock->GetStreamTime());
		} else if (blinkerStatus == BlinkerHazard) {
			SEND_WRAPPED_SAMPLE(blinkerHazardPin, val, _clock->GetStreamTime());
		}
	}
	blinkerStatus = value;
	RETURN_NOERROR;
}

tResult PathPlanner::setManeuverResponse(ManeuverResponse r)
{
	uint8_t val = static_cast<uint8_t>(r);
	SEND_MEDIA_SAMPLE(maneuverResponsePin, uint8_t, val, _clock->GetStreamTime());
	RETURN_NOERROR;
}

tResult PathPlanner::resetLaneDetector()
{
	uint8_t val = 0;
	SEND_MEDIA_SAMPLE(resetLaneTrackingPin, uint8_t, val, _clock->GetStreamTime());
	RETURN_NOERROR;
}

void PathPlanner::setCurrentManeuver(ManeuverType m)
{
	logger->log() << "setCurrentManeuver " << m << endl;
	// Clear data
	crossings.clear();
	streetSigns.clear();
	switch (m) {
	// Parking
	case ManeuverParallelParking: // Fall through
	case ManeuverCrossParking:
	{
		logger->GLogTrap("Waiting for parking sign");
		state = WaitForParkingSign;
		if (!shouldLaneFollow) {
			enableLaneTracking();
		}
		setBlinker(BlinkerOff);
		break;
	}
	case ManeuverPulloutRight:
	{
		ControllerCommand p;
		p.pose = currPose;
		if (currentManeuver == ManeuverParallelParking) {
			p.type=PullOutParallel;
			setBlinker(BlinkerLeft);
			p.pose.speed = 0; // Unused
			p.cruiseVelocity = 0; // Unused
			sendDesiredPose(p);
		} else if (currentManeuver == ManeuverCrossParking ) {
			p.type=PullOutCrossRight;
			setBlinker(BlinkerRight);
			p.pose.speed = 0; // Unused
			p.cruiseVelocity = 0; // Unused
			sendDesiredPose(p);
		} else {
			logger->GLogTrap("Requesting state...");
			requestInitialState();
			return;
		}
		break;
	}
	case ManeuverPulloutLeft:
	{
		ControllerCommand p;
		p.pose = currPose;
		p.type=PullOutCrossLeft;
		p.pose.speed = 0; // Unused
		p.cruiseVelocity = 0; // Unused
		setBlinker(BlinkerLeft);
		sendDesiredPose(p);
		break;
	}
	case ManeuverLeft:
	case ManeuverRight:
	case ManeuverStraight:
	{
		state = WaitForCrossing;
		setBlinker(BlinkerOff);
		enableLaneTracking();
		break;
	}
	case ManeuverComplete:
	{
		setBlinker(BlinkerHazard);
		[[clang::fallthrough]];
	}
	case ManeuverIdle:
	{
		state = Ready;
		disableLaneTracking();
		ControllerCommand p;
		p.pose = currPose;
		p.pose.speed = 0;
		p.cruiseVelocity = 0;
		if (m == ManeuverIdle) {
			setBlinker(BlinkerOff);
		}
		sendDesiredPose(p);
		break;
	}
	}
	currentManeuver = m;
}

void PathPlanner::updateTimer()
{
	if (timerRunning && _clock->GetStreamTime() - waitStart > waitPeriod) {
		timerRunning = false;
		if (state == ParkingWaitPeriod) {
			state = Ready;
			setBlinker(BlinkerOff);
			setManeuverResponse(ManeuverResponseComplete);
		}
	}
}

void PathPlanner::enableLaneTracking()
{
	path.resetPath(currPose);
	path.processLaneData(lastLaneData, currPose, debugDrivingPath ? this : NULL, debugDrivingPath ? logger : NULL);
	resetLaneDetector();
	shouldLaneFollow = true;
	requestLaneTrackingPose();
	logger->GLogTrap("Lane following enabled");
}

void PathPlanner::disableLaneTracking()
{
	shouldLaneFollow = false;
	poseRequested = false;
	resetLaneDetector();
	logger->GLogTrap("Lane following disabled");
}

void PathPlanner::requestLaneTrackingPose()
{
	poseRequested = true;
}

void PathPlanner::checkForCrossing()
{
	if (isCrossingManeuver() && state == WaitForCrossing) {
		if (crossings.empty()) {
			// No crossings
			return;
		}
		for (size_t i = 0; i < crossings.size(); ++i) {
			if (crossings[i].numDetections > minCrossingDetections) {
				if (norm(crossings[i].center - getOdometryPos(currPose)) > maxCrossingDist) {
					logger->log() << "Crossing at " << crossings[i].center << "ignored, too far away" << endl;
					continue;
				}
				logger->log() << "Crossing at " << crossings[i].center << ", car is at " << currPose << ", driving to stop line for maneuver " << currentManeuver << endl;
				// TODO check signs to know right of way and stuff
				disableLaneTracking();
				crossingStopCommand = getCrossingStop(crossings[i], crossingStopOffset);
				crossingStopCommand.cruiseVelocity = lfSpeed;
				RoadSign s = getRoadSign(Point2d(crossingStopCommand.pose.x, crossingStopCommand.pose.y));
				crossingHaveway = checkHaveway(s);
				logger->log() << "Sign is " << s << " have way? " << crossingHaveway << endl;
				if (!checkObstacles) {
					// Force bypass
					crossingHaveway = true;
				}
				if (crossingHaveway) {
					crossingStopCommand.pose.speed = lfSpeed;
				} else {
					setCrossingCheckDirs(s, crossings[i]);
					if (!crossingDirsToCheck.f && !crossingDirsToCheck.l && !crossingDirsToCheck.r) {
						crossingHaveway = true;
						crossingStopCommand.pose.speed = lfSpeed;
					}
				}
				if (s == RoadSignStopandgiveway) {
					// If it's a stop sign, stop even if we have right of way.
					crossingStopCommand.pose.speed = 0;
				}
				crossingExitCommand = getCrossingExit(crossings[i], currentManeuver, getCrossingPoseFwdOffset(), getCrossingPoseCrossOffset());
				crossingExitCommand.cruiseVelocity = crossingTurnVel;
				crossingExitCommand.pose.speed = crossingTurnVel;
				state = WaitForCrossingStop;
				if (currentManeuver == ManeuverRight) {
					setBlinker(BlinkerRight);
				} else if (currentManeuver == ManeuverLeft) {
					setBlinker(BlinkerLeft);
				}
				sendDesiredPose(crossingStopCommand);
			}
		}
	}
}

RoadSign PathPlanner::getRoadSign(const Point2d &p)
{
	for (size_t i = 0; i < streetSigns.size(); ++i) {
		Point2d sp(streetSigns[i].s.pos.x, streetSigns[i].s.pos.y);
		if (norm(sp - p) < .5) {
			return streetSigns[i].s.id;
		}
	}
	return RoadSignNomatch;
}

bool PathPlanner::checkHaveway(RoadSign s)
{
	if (s == RoadSignHaveway && currentManeuver != ManeuverLeft) {
		// if Have Way sign is present we can go, except when turning left
		return true;
	}
	if (currentManeuver == ManeuverRight && (s == RoadSignNomatch || s == RoadSignUnmarkedintersection)) {
		// if we turn right and there's no roadsign or the intersection is unmarked, we
		// don't have to check
		return true;
	}
	// Any other time we have to check for traffic
	return false;
}

int PathPlanner::mergeSign(const SignStruct &data)
{
	for (size_t i = 0; i < streetSigns.size(); ++i) {
		if (streetSigns[i].s.id == data.id && norm(data.pos - streetSigns[i].s.pos) < .5) {
			streetSigns[i].s.pos = streetSigns[i].s.pos * streetSigns[i].hits + data.pos;
			++streetSigns[i].hits;
			streetSigns[i].s.pos /= streetSigns[i].hits;
			return streetSigns[i].hits;
		}
	}
	MergedSign ms;
	ms.s = data;
	ms.hits = 1;
	streetSigns.push_back(ms);
	return 1;
}

tResult PathPlanner::debugDrawEverything(const LaneData &ld)
{
	drawLine(ld.rightLane, CV_RGB(255, 0, 0));
	drawLine(ld.leftLane, CV_RGB(255, 0, 0));
	drawLine(ld.centerLane, CV_RGB(0, 255, 0));
	drawLine(*path.getPath(), CV_RGB(0, 255, 255), 2);
	circle(debugImg, toImg(getOdometryPos(currPose)), 3, CV_RGB(255, 255, 255));
	cObjectPtr<IMediaSample> sample;
	RETURN_IF_FAILED(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &sample));
	RETURN_IF_FAILED(sample->AllocBuffer(outFormat.nSize));
	sample->Update(0, debugImg.data, outFormat.nSize, 0);
	RETURN_IF_FAILED(videoOut.Transmit(sample));
	debugImg *= debugDrawPersistency; // darken prev image
	RETURN_NOERROR;
}

Point2i PathPlanner::toImg(const Point2d &p) const
{
	return Point2i(debugimgcenter + p.x * debugDrawScale, debugimgcenter - p.y * debugDrawScale);
}

double PathPlanner::getCrossingPoseFwdOffset()
{
	if (currentManeuver == ManeuverRight) {
		return crossingPoseOffsetFwdRight;
	} else if (currentManeuver == ManeuverLeft) {
		return crossingPoseOffsetFwdLeft;
	}
	return 0;
}

double PathPlanner::getCrossingPoseCrossOffset()
{
	if (currentManeuver == ManeuverRight) {
		return crossingPoseOffsetCrossRight;
	} else if (currentManeuver == ManeuverLeft) {
		return crossingPoseOffsetCrossLeft;
	}
	return 0;
}

void PathPlanner::setCrossingCheckDirs(RoadSign s, const CrossingDetectionEvent &e)
{
	// Not considered: traffic that turns left/right and has right of way
	crossingDirsToCheck.f = false;
	crossingDirsToCheck.l = false;
	crossingDirsToCheck.r = false;
	if (currentManeuver == ManeuverStraight) {
		if (s == RoadSignStopandgiveway || s == RoadSignGiveway) {
			crossingDirsToCheck.l = HAS_EXIT_LEFT(e);
		}
		crossingDirsToCheck.r = HAS_EXIT_RIGHT(e);
	} else if (currentManeuver == ManeuverRight) {
		if (s == RoadSignStopandgiveway || s == RoadSignGiveway) {
			crossingDirsToCheck.l = HAS_EXIT_LEFT(e);
		}
	} else if (currentManeuver == ManeuverLeft) {
		if (s == RoadSignHaveway) {
			// have way, only need to check oncoming traffic
			crossingDirsToCheck.f = HAS_EXIT_STRAIGHT(e);
		} else {
			// Check right because right has right of way
			// Check straight because has right of way
			crossingDirsToCheck.f = HAS_EXIT_STRAIGHT(e);
			crossingDirsToCheck.r = HAS_EXIT_RIGHT(e);
			if (s == RoadSignStopandgiveway || s == RoadSignGiveway) {
				// Additionally check left if any because we must yield all traffic.
				crossingDirsToCheck.l = HAS_EXIT_LEFT(e);
			}
		}
	}
	logger->log() << "Traffic to check for: l: " << crossingDirsToCheck.l << " r: " << crossingDirsToCheck.r << " f: " << crossingDirsToCheck.f
				  << ", exits available were " << hex << static_cast<int>(e.exits) << dec << endl;
}

tResult PathPlanner::sendCrossingLeftIntermediateCommand()
{
	logger->GLogTrap("Executing crossing left turn intermediate maneuver");
	state = WaitForCrossingLeftIntermediate;
	Point2d p = getOdometryPos(crossingStopCommand.pose) + crossingStopLeftIntermediateOffs * getOdometryDirection(crossingStopCommand.pose);
	ControllerCommand c;
	c.pose.x = p.x;
	c.pose.y = p.y;
	c.pose.theta = crossingStopCommand.pose.theta;
	c.pose.speed = crossingTurnVel;
	c.cruiseVelocity = crossingTurnVel;
	c.type = DesiredPose;
	return sendDesiredPose(c);
}

bool PathPlanner::rightIsBlocked()
{
	return currBlockedSpots.right_lane;
}

bool PathPlanner::leftIsBlocked()
{
	// Don't check
	return false;
}

bool PathPlanner::straightIsBlocked()
{
	return currBlockedSpots.straight_lane;
}

void PathPlanner::drawLine(const vector<Point2d> &l, Scalar col, int thickness)
{
	if (l.empty()) {
		return;
	}
	if (l.size() < 2) {
		cv::circle(debugImg, toImg(l[0]), thickness, col, thickness);
	}
	for (size_t i = 0; i < l.size() - 1; ++i) {
		cv::line(debugImg, toImg(l[i]), toImg(l[i + 1]), col, thickness);
	}
}

void PathPlanner::drawLine(const Point2d &p, const Point2d &q, Scalar col, int thickness)
{
	line(debugImg, toImg(p), toImg(q), col, thickness);
}

void PathPlanner::drawArrowedLine(const Point2d &p, const Point2d &dir, Scalar col)
{
	Point2d pp = toImg(p);
	arrowedLine(debugImg, pp, pp + dir, col);
}

tResult PathPlanner::requestInitialState()
{
	state = WaitingForInitialState;
	uint8_t dummy = 0;
	SEND_MEDIA_SAMPLE(requestInitialStatePin, uint8_t, dummy, _clock->GetStreamTime());
	RETURN_NOERROR;
}

bool PathPlanner::isParking() const
{
	return currentManeuver == ManeuverCrossParking || currentManeuver == ManeuverParallelParking;
}

bool PathPlanner::isPullout() const
{
	return currentManeuver == ManeuverPulloutLeft || currentManeuver == ManeuverPulloutRight;
}

bool PathPlanner::isCrossingManeuver() const
{
	return currentManeuver == ManeuverStraight || currentManeuver == ManeuverLeft || currentManeuver == ManeuverRight;
}

Scalar cols[] = {CV_RGB(255, 0, 0), CV_RGB(0, 0, 255), CV_RGB(0, 255, 0), CV_RGB(255, 255, 0), CV_RGB(0, 255, 255)};

void PathPlanner::drivingPathDebugDrawFunc(const std::vector<Point2d> &v, const std::vector<float> &w)
{
	Point2d avgP = getAvgPoint(v, w);
	for (size_t i = 0; i < v.size(); ++i) {
		if (w[i] != 0) {
			drawLine(avgP, v[i], cols[i]);
		}
	}
}

void PathPlanner::drivingPathDebugDrawCircle(const Point2d &p, int r, Scalar col)
{
	circle(debugImg, toImg(p), r, col);
}

