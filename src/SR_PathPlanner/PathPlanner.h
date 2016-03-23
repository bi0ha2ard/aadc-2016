#ifndef SR_PATHPLANNER_H_
#define SR_PATHPLANNER_H_

#include <odometry_data.h>
#include <map_data.h>
#include "../misc/PinWrapper.h"
#include "../misc/SR_Logger.h"
#include <maneuverdata.h>
#include "DrivingPath.h"
#include <adtf_plugin_sdk.h>

#define OID_SR_PATHPLANNER "sr.pathplanner"

class PathPlanner : public adtf::cFilter, public DrivingPathDebugDrawer
{
	ADTF_FILTER(OID_SR_PATHPLANNER, "SpaceRacer Path Planner", adtf::OBJCAT_Application)

public:
	PathPlanner(const char *name);
	virtual ~PathPlanner();
	tResult PropertyChanged(const tChar *strName);

protected:
	tResult Init(tInitStage stage, __exception);
	tResult Shutdown(tInitStage stage, __exception);
	tResult OnPinEvent(adtf::IPin *source, tInt eventCode, tInt param1, tInt param2, adtf::IMediaSample *mediaSample);
	tResult Start(__exception);

private:
	enum BlinkerValue {
		BlinkerOff,
		BlinkerLeft,
		BlinkerRight,
		BlinkerHazard
	};

	tResult doParking(const OdometryPose &p);
	tResult requestParkingSearch(bool parallel);
	tResult handlePoseReached();
	void handleKeyboardCommand(float command);
	void handleInitialState(InitialCarConfiguration c);
	tResult sendDesiredPose(const cv::Point2d &pos, const cv::Point2d dir, double speed = 1, float cruiseSpeed = 1);
	tResult sendDesiredPose(const ControllerCommand &p);
	tResult processNewPose();
	tResult processLaneData(LaneData &data);
	void processCrossingData(CrossingDetectionEvent &data);
	tResult processObstacles();
	tResult processSigns(SignStruct &data);
	tResult setBlinker(BlinkerValue value);
	tResult setManeuverResponse(ManeuverResponse r);
	tResult resetLaneDetector();
	void setCurrentManeuver(ManeuverType m);
	void updateTimer();
	void enableLaneTracking();
	void disableLaneTracking();
	void requestLaneTrackingPose();
	void checkForCrossing();
	RoadSign getRoadSign(const Point2d &p);
	bool checkHaveway(RoadSign s);
	int mergeSign(const SignStruct &data);
	tResult debugDrawEverything(const LaneData &ld);
	inline Point2i toImg(const Point2d &p) const;
	double getCrossingPoseFwdOffset();
	double getCrossingPoseCrossOffset();
	void setCrossingCheckDirs(RoadSign s, const CrossingDetectionEvent &e);
	tResult sendCrossingLeftIntermediateCommand();
	bool rightIsBlocked();
	bool leftIsBlocked();
	bool straightIsBlocked();
	void drawLine(const vector<Point2d> &line, Scalar col, int thickness = 1);
	void drawLine(const Point2d &p, const Point2d &q, Scalar col, int thickness  = 1);
	void drawArrowedLine(const Point2d &p, const Point2d &dir, Scalar col);
	tResult requestInitialState();

	bool isParking() const;
	bool isPullout() const;
	bool isCrossingManeuver() const;

	adtf::cInputPin posePin;
	adtf::cInputPin crossingPin;
	adtf::cInputPin lanePin;
	adtf::cInputPin parkingReferencePoint;
	adtf::cInputPin blockedSpotsPin;
	adtf::cInputPin signPin;
	adtf::cInputPin maneuverPin;
	WrappedBVInputPin parkingBlinker;
	WrappedSVInputPin keyboardCommandPin;
	WrappedBVInputPin controllerPoseReachedPin;
	adtf::cInputPin initialStatePin;
	bool activeParking;
	int minParkingSignHits;

	BlockedSpots currBlockedSpots;

	adtf::cOutputPin controllerOutputPin;
	adtf::cOutputPin maneuverResponsePin;
	WrappedBVOutputPin parkingSearchRequestPin;
	adtf::cOutputPin resetLaneTrackingPin;
	WrappedBVOutputPin blinkerRightPin;
	WrappedBVOutputPin blinkerLeftPin;
	WrappedBVOutputPin blinkerHazardPin;
	adtf::cVideoPin videoOut;
	adtf::cOutputPin requestInitialStatePin;

	OdometryPose currPose;
	cv::Point2d currHeading;

	Mat debugImg;
	tBitmapFormat outFormat;
	int debugDrawScale;
	float debugDrawPersistency;

	ManeuverType currentManeuver;
	enum PlannerState {
		Ready,
		WaitingForInitialState,
		WaitForCrossing,
		WaitForCrossingStop,
		WaitForCrossingTraffic,
		WaitForCrossingLeftIntermediate,
		WaitForCrossingManeuverEnd,
		WaitForParkingSign,
		WaitForParkingSpotScanStart,
		WaitForParkingSpot,
		ExecutingParkingManeuver,
		ParkingWaitPeriod,
	} state;

	struct MergedSign {
		SignStruct s;
		int hits;
	};

	struct {
		bool l;
		bool r;
		bool f;
	} crossingDirsToCheck;

	vector<MergedSign> streetSigns;

	BlinkerValue blinkerStatus;
	tTimeStamp waitStart;
	tTimeStamp waitPeriod;
	bool timerRunning;

	SR_Logger *logger;
	bool debugDrivingPath;
	bool debugPoseRequests;

	bool shouldLaneFollow;
	bool poseRequested;
	double lfCommandDist;
	double lfUpdateDist;
	float lfSpeed;
	float parkScanSpeed;
	DrivingPath path;
	Point2d lastCommandPos;
	ControllerCommand crossingStopCommand;
	ControllerCommand crossingExitCommand;
	LaneData lastLaneData;
	bool useInterpCommand;
	bool crossingHaveway;
	std::vector<CrossingDetectionEvent> crossings;
	int minCrossingDetections;
	float maxCrossingDist;
	bool checkObstacles;
	int freeCounter;
	int minFreeTicks;
	float crossingTurnVel;
	double crossingPoseOffsetFwdLeft;
	double crossingPoseOffsetFwdRight;
	double crossingPoseOffsetCrossLeft;
	double crossingPoseOffsetCrossRight;
	double crossingStopOffset;
	double crossingStopLeftIntermediateOffs;

	// DrivingPathDebugDrawer interface
public:
	void drivingPathDebugDrawFunc(const std::vector<Point2d> &v, const std::vector<float> &w);
	void drivingPathDebugDrawCircle(const Point2d &p, int r, Scalar col);
};


#endif
