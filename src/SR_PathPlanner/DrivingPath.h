#ifndef DRIVINGPATH_H
#define DRIVINGPATH_H

#include <opencv/cv.h>
#include <map_data.h>
#include <odometry_data.h>
using namespace cv;
using namespace std;

class SR_Logger;

class DrivingPathDebugDrawer
{
public:
	virtual ~DrivingPathDebugDrawer();
	virtual void drivingPathDebugDrawFunc(const std::vector<cv::Point2d> &v, const std::vector<float> &w) = 0;
	virtual void drivingPathDebugDrawCircle(const Point2d &p, int r, Scalar col) = 0;
};

class DrivingPath
{
	friend class DrivingPathTest;
public:
	DrivingPath();
	ControllerCommand getNextCommand(const OdometryPose &p, double desiredDistance = 1.0, SR_Logger *logger = NULL) const;
	ControllerCommand getNextCommandInterp(const OdometryPose &p, double desiredDistance = 1.0, SR_Logger *logger = NULL) const;
	void processLaneData(const LaneData &l, const OdometryPose &currPose, DrivingPathDebugDrawer *thread = NULL, SR_Logger *logger = NULL);
	const vector<Point2d> *getPath() const;
	OdometryPose getPoseFromIdx(const OdometryPose &currPose, size_t idx) const;
	OdometryPose getInterpPose(size_t i, double dist) const;
	void resetPath(const OdometryPose &currPose);
	bool empty() const;

	Point2d getExtrapolatedPoint(size_t idx) const;

	void setAgeBasedWeighting(bool value);
	void setInitialPointWeight(float value);
	void setWeightIncrement(float value);
	void setRejectionDistance(double value);
	void setStepSize(double value);

	bool getAgeBasedWeighting() const;
	float getInitialPointWeight() const;
	float getWeightIncrement() const;
	double getRejectionDistance() const;
	double getStepSize() const;

	int getNumPoints() const;
	void setNumPoints(int value);

	float getWeightUpdExtr() const;
	void setWeightUpdExtr(float value);

	float getWeightUpdRight() const;
	void setWeightUpdRight(float value);

	float getWeightUpdLeft() const;
	void setWeightUpdLeft(float value);

	float getWeightUpdCenter() const;
	void setWeightUpdCenter(float value);

	float getWeightExtrExtr() const;
	void setWeightExtrExtr(float value);

	float getWeightExtrRight() const;
	void setWeightExtrRight(float value);

	float getWeightExtrLeft() const;
	void setWeightExtrLeft(float value);

	float getWeightExtrCenter() const;
	void setWeightExtrCenter(float value);

private:
	/**
	 * @brief pointBehind
	 * @param i
	 * @param p
	 * @return  true if p infront of point path[i - 1]
	 */
	bool pointBehind(size_t i, const Point2d &p);
	void eraseItem(size_t i);
	void appendItem(const Point2d &p);
	void updateItem(size_t i, const Point2d &p);
	int eraseOldPoints(const Point2d &pose);
	int fillInLane(const LaneData &ld, int lane, DrivingPathDebugDrawer *thread, SR_Logger *logger);

	vector<Point2d> path;
	vector<float> pathWeights;

	bool ageBasedWeighting;
	float initialPointWeight;
	float weightIncrement;
	double rejectionDistance;
	double stepSize;
	int numPoints;

	float weightUpdExtr;
	float weightUpdRight;
	float weightUpdLeft;
	float weightUpdCenter;
	float weightExtrExtr;
	float weightExtrRight;
	float weightExtrLeft;
	float weightExtrCenter;
};

#endif // DRIVINGPATH_H
