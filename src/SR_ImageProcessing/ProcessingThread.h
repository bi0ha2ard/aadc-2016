#ifndef SR_PROCESSING_THREAD_H_
#define SR_PROCESSING_THREAD_H_


#include <opencv/cv.h>
#include <adtf_plugin_sdk.h>
#include <ctime>
#include "SR_ImageProcessing.h"
#include <odometry_data.h>

// DEBUG!
#include "../SR_PathPlanner/DrivingPath.h"

class SR_ImageProcessing;

class ProcessingThread : public adtf::IKernelThreadFunc, public DrivingPathDebugDrawer
{
public:
	enum LaneSearchType {
		RightLaneSearch,
		LeftLaneSearch
	};

	ProcessingThread(const char *name, SR_ImageProcessing *imageProcessing);
	virtual ~ProcessingThread();

	// To be called by caller
	void requestProcessing(adtf::IMediaSample *s, tTimeStamp timeStamp, OdometryPose p);
	tResult createThread();
	tResult shutdownThread();
	tResult stopThread();
	tResult runThread();
	void testExtrapolation();
	void resetLaneTracking();
	void resetLaneTracking(LaneSearchType type);

	void drivingPathDebugDrawFunc(const std::vector<cv::Point2d> &v, const std::vector<float> &w);
	void drivingPathDebugDrawCircle(const Point2d &p, int r, Scalar col);

protected:
	tResult ThreadFunc(adtf::cKernelThread *pThread, tVoid *pvUserData, tSize szUserData);

private:
	enum DetectedType {
		Garbage,
		Center,
		RightLaneCurve,
		LeftLaneCurve,
		Crossing
	};

	typedef struct
	{
		size_t contour_idx;
		double radiusEstimate;
		double cArea;
		double aspectRatio;
		float longSide;
		float shortSide;
		cv::RotatedRect rect;
		cv::Point2f curveCenter;
		cv::Point2f vecToCenter;
		DetectedType type;
	} Feature;
	
	static bool featureLowerThan(const Feature *a, const Feature *b);

	typedef struct
	{
		cv::Point center;
		bool isBig;
	} CurveDescription;

	typedef struct
	{
		uint8_t exits;
		uint8_t blockedExits;
		bool stopLineOnTrack;
		cv::Point2d crossingCenter;
		cv::Point2d crossingNormal;
	} CrossingCandidate;

	std::vector<CrossingCandidate> crossingCandidates;

	tResult process();
	tResult processCanny();
	static Feature generateFeature(const vector<vector<cv::Point> > &contours, size_t i);
	void resetLaneData();
	bool isCenterLine(const Feature &f) const;
	bool findCurve(const vector<cv::Point> &contour, Feature &f);
	bool findCorners(Feature &f, vector<vector<cv::Point> > &contours);
	CrossingCandidate findCrossingExitsFromCorner(const cv::Point2d &anchor, cv::Point2d normal, const cv::Point2d &l1back, const Point2d &l2back);
	uint8_t findCrossingExitsFromImage(CrossingCandidate &cc);
	tResult findParkingSlot();
	int distToWhite(const cv::Point2d &anchor, const cv::Point2d &normal, int start, int max, bool &outsideImageRegion) const;
	bool containsLanes(cv::Mat area) const;
	bool insideImageRegion(const cv::Point &p) const;
	bool insideCrossingRegion(const cv::Point &p) const;
	void extractCurves(vector<CurveDescription> &curves, const vector<Feature *> &features);
	void extractCrossings();
	void fitCenterLine();
	void findLeftRightLanes();
	std::vector<cv::Point2i> getLaneEstimate(const cv::Point2i &_start, const cv::Point2d &_normal, const std::vector<cv::Point2i> &oldLane, int stepsize = 10, size_t num = 40) const;
	std::vector<cv::Point2i> extractLane(const cv::Point2i &_start, const LaneSearchType type, const std::vector<cv::Point2i> &oldLane, bool debugDraw, Point2d *defSearchNorm = NULL);
	vector<Point2i> extractCenterLane();
	/*static*/ void getNextSearchPoint(const cv::Point2d &a, const cv::Point2d &b, LaneSearchType type, cv::Point2i &currSearchPoint, cv::Point2d &currSearchNormal, bool noCheck = false, bool draw = false);
	/*static*/ cv::Point2d getExtrapolatedPoint(const std::vector<cv::Point2i> &lane, const cv::Point2d &currSearchPoint, const cv::Point2d &currSearchNormal);
	void doLineFits(const vector<cv::Point2i> &right);
	bool pointOnStreet(const cv::Point &p) const;
	void updateOldLanes();
	void checkLaneReset();
	void generateAndSendLaneData();
	void checkCenterFeature(Feature &f);
	void debugDrawFeatures();
	void drawDrivingPath();
	void debugDrawCrossingExit(const ControllerCommand &c, Scalar col);
	void updateDrivingPath(const LaneData &l);
	cv::Point2d makeStopLineCrossingNormal(const Point2d &anchor, const Point2d &l2back) const;
	bool hasStopLine(const std::vector<cv::Point2i> &closeLine, const std::vector<cv::Point2i> &centerLine, const std::vector<cv::Point2i> &farLine, const Point2d &anchor);
	void setConstants();

	volatile bool newData;
	volatile bool shouldrun;
	adtf::cKernelEvent event;
	cKernelThread thread;
	cv::Mat data;
	tTimeStamp currTimeStamp;
	cMutex dataMutex;
	cString name;
	cString logName;
	SR_ImageProcessing *imageProcessing;
	char *imgBuf;

	// Timing
	timespec prevStartTime;
	timespec startTime;
	timespec endTime;

	std::vector<cv::Point2i> validCrossingRegion;

	// opencv members for line detection
	cv::Mat dilated;
	cv::Mat m_matLineCannyUncut;
	cv::Mat m_ImageUncut;
	cv::Mat m_matGreyUncut;
	cv::Mat m_matGreyThreshUncut;
	cv::Mat m_TopDown;
	cv::Mat m_Drawing;
	cv::Mat m_Output;

	SR_ImageProcessing::ImageProcPrefs prefs;

	std::vector<Feature> prevFeatures;
	std::vector<Feature> currFeatures;
	std::vector<Feature*> centerLane;

    vector<cv::Point2i> right;
	vector<cv::Point2i> left;
	vector<cv::Point2i> center;
	std::vector<cv::Point2d> worldLaneRight;
	std::vector<cv::Point2d> worldLaneLeft;
	std::vector<cv::Point2d> worldLaneCenter;
	vector<Point2i> oldRight;
	vector<Point2i> oldLeft;
	vector<Point2i> oldCenter;

	volatile bool shouldResetRight;
	volatile bool shouldResetLeft;
	cv::Mutex resetMutex;


	// Constants for this run
	double laneCenterLPix;
	double laneCenterLDev;
	float crossingCornerL1Pix;
	float crossingCornerL2Pix;
	double laneDistToPath;
	double l10cm;
	double l60cm;
	double l20cm;
	double l50cm;
	double l1m;
	double crossingCenterDist;
	double crossingExitMinFreeDist;
	double crossingExitMaxDist;

	cv::Vec4f centerLine;
	int centerLineAge;
	const int centerLineMaxAge;
	OdometryPose lastCenterPose;
	double centerLineAngleDelta;

	OdometryPose lastPose;

	OdometryPose currentPose;
	// image to world transform data
	double a1,b1,c1;
	double a2,b2,c2;
	double a3,b3,c3;
	double a4,b4,c4;
	void updateTransform();
	cv::Point2d imgToWorld(const cv::Point &p) const;
	cv::Point2i worldToImg(const cv::Point2d &p) const;
	void transformLineToWorld(const std::vector<cv::Point> &input, std::vector<cv::Point2d> &output) const;
	void transformLineToImg(const std::vector<cv::Point2d> &input, std::vector<cv::Point2i> &output) const;
	void testTransforms() const;

	static double distFromCarSqrt(const cv::Point &p);
	static int containsCurve(const vector<CurveDescription> &curves, const Feature &f);
	static void getDistanceFromSide(const std::vector<cv::Point> &contour, const Feature &f, double *inner, double *outer, cv::Point2f *normal = NULL);
	void drawLine(const std::vector<cv::Point> &line, cv::Scalar colour, int thickness = 1, int lineType = CV_AA);

	/**
	 * @brief smoothCurve
	 * Smooth a curve
	 * Algorithm taken from openFramework
	 * smoothingSize is the size of the smoothing window. So if
	 * smoothingSize is 2, then 2 points from the left, 1 in the center,
	 * and 2 on the right (5 total) will be used for smoothing each point.
	 * smoothingShape describes wheter to use a triangular window (0) or
	 * box window (1) or something in between (for example, .5).
	 * @param curve input curve
	 * @param output smoothed curve output
	 * @param smoothingSize
	 * @param smoothingShape
	 */
	static void smoothCurve(const std::vector<cv::Point2i> &curve, std::vector<cv::Point2i> &output, int smoothingSize, float smoothingShape);

	template <typename T>
	static T clamp(const T& n, const T& lower, const T& upper) {
		return std::max(lower, std::min(n, upper));
	}

	DrivingPath path;
	std::vector<cv::Point2i> imgPath;
};


#endif
