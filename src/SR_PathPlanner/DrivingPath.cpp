#include "DrivingPath.h"
#include "../misc/LineHelpers.h"
#include "../SR_ImageProcessing/ProcessingThread.h"
#include "../misc/SR_Logger.h"
#include "../misc/OdometryHelpers.h"

const double rightLaneOffset = .22;
const double leftLaneOffset = .47 + .22;
const double centerLaneOffset = .22;

DrivingPath::DrivingPath()
{
	// Start with 4m straight line
	OdometryPose p;
	p.x = 0;
	p.y = 0;
	p.theta = 0;
	stepSize = .2f;
	numPoints = 20;
	ageBasedWeighting = true;
	initialPointWeight = 1.0f;
	weightIncrement = 1.0f;
	rejectionDistance = 1.0f;
	weightUpdExtr = .5f;
	weightUpdRight = 2;
	weightUpdLeft = 1.2f;
	weightUpdCenter = 1.2f;
	weightExtrExtr = .5f;
	weightExtrRight = 3;
	weightExtrLeft = 2;
	weightExtrCenter = 2;
	resetPath(p);
}

ControllerCommand DrivingPath::getNextCommand(const OdometryPose &p, double desiredDistance, SR_Logger *logger) const
{
	// Point 1m ahead
	Point2d currP(p.x, p.y);
	Point2d nextP = currP + getOdometryDirection(p) * desiredDistance;
	double minDist = 9999;
	ssize_t pos = -1;
	for (size_t i = 0; i < path.size(); ++i) {
		double currDist;
		if ((currDist = norm(nextP - path[i])) < minDist) {
			minDist = currDist;
			pos = i;
		}
	}
	ControllerCommand c;
	if (pos >= 0 && orthoProject(path[pos], currP, nextP) > 0) {
		c.pose = getPoseFromIdx(p, pos);
		// TODO
		c.pose.speed = 1;
		c.cruiseVelocity = 1;
		c.type = DesiredPose;
		if (logger) {
			logger->log() << "Found pose " << c << " at index " << pos << " from current pose " << p << ", orthoproject was " << orthoProject(path[pos], currP, nextP) << endl;
		}
		return c;
	} else {
		// Error
		c.pose = p;
		c.pose.speed = 0;
		c.cruiseVelocity = 0;
		c.type = DesiredPose;
		if (logger) {
			Point2d dummy(-999, -999);
			logger->log() << "Failed to find next pose, curr pose " << currP << " path size " << path.size() << " first " << (path.empty() ? dummy : path.front()) << " last " << (path.empty() ? dummy : path.back())
						  << " idx was " << pos << " ortho project was " << orthoProject(path[pos], currP, nextP) << endl;
		}
	}
	return c;
}

ControllerCommand DrivingPath::getNextCommandInterp(const OdometryPose &p, double desiredDistance, SR_Logger *logger) const
{
	Point2d currP(p.x, p.y);
	double minDist = 9999;
	ssize_t pos = -1;
	for (size_t i = 0; i < path.size(); ++i) {
		double currDist;
		if ((currDist = norm(currP - path[i])) < minDist) {
			minDist = currDist;
			pos = i;
		}
	}

	ControllerCommand c;
	if (pos >= 0 && (pos + 1) < path.size() && minDist < desiredDistance) {
		double remainingDist = desiredDistance - minDist;
		c.pose = getPoseFromIdx(p, path.size() - 1);
		for (size_t i = pos + 1; i < path.size(); ++i) {
			double nn = norm(path[i] - path[i - 1]);
			if (nn < remainingDist) {
				remainingDist -= nn;
			} else {
				c.pose = getInterpPose(i - 1, remainingDist);
				break;
			}
		}
		c.pose.speed = 1;
		c.cruiseVelocity = 1;
		c.type = DesiredPose;
		if (logger) {
			logger->log() << "Found pose " << c << " at index " << pos << " from current pose " << p << endl;
		}
		return c;
	} else {
		// Error
		c.pose = p;
		c.pose.speed = 0;
		c.cruiseVelocity = 0;
		c.type = DesiredPose;
		if (logger) {
			logger->log() << "Failed to get pose" << endl;
		}
	}
	return c;
}

void DrivingPath::processLaneData(const LaneData &l, const OdometryPose &currPose, DrivingPathDebugDrawer *thread, SR_Logger *logger)
{
	int removed = 0;
	int removed2 = 0;
	int adjusted = 0;
	int extrapolatedPoints = 0;
	int skipped = 0;
	Point2d pose(currPose.x, currPose.y);
	removed = eraseOldPoints(pose);
	if (path.empty()) {
		resetPath(currPose);
	}

	// TODO keep old lane data?
	// fuse real point, interpolated point, points generated from
	// right, left, center lane
	vector<float> weights(5);
	vector<Point2d> points(5);
	for (size_t i = 0; i < path.size(); ++i) {
		points[0] = path[i];
		points[1] = getExtrapolatedPoint(i);
		double distToExt = norm(path[i] - points[1]);
		weights[0] = fmax(0, (1 - fabs(distToExt - stepSize) * .5));
		if (ageBasedWeighting) {
			weights[0] += pathWeights[i];
		}
		weights[1] = weightUpdExtr;
		points[2] = getCorrespPoint(l.rightLane, points[0], true, rightLaneOffset, weights[2]);
		weights[2] *= weightUpdRight;
		points[3] = getCorrespPoint(l.leftLane, points[0], false, leftLaneOffset, weights[3]);
		weights[3] *= weightUpdLeft;
		points[4] = getCorrespPoint(l.centerLane, points[0], false, centerLaneOffset, weights[4]);
		weights[4] *= weightUpdCenter;
		if (thread) {
			thread->drivingPathDebugDrawFunc(points, weights);
		}
		Point2d newPoint = getAvgPoint(points, weights);
		if (pointBehind(i, newPoint) || (i > 0 && !pointContinuesLane(path, newPoint, i - 1))) {
			// skip backwards stuff
			eraseItem(i);
			--i;
			++removed2;
		} else {
			updateItem(i, newPoint);
			++adjusted;
		}
	}

	if (thread && !path.empty()) {
		thread->drivingPathDebugDrawCircle(path.back(), 10, CV_RGB(255, 0, 0));
	}
	// right
	extrapolatedPoints +=  fillInLane(l, 2, thread, logger);
	// left
	extrapolatedPoints += fillInLane(l, 3, thread, logger);
	// center
	extrapolatedPoints += fillInLane(l, 4, thread, logger);

	if (extrapolatedPoints > 0) {
		// trigger extrapolation if we filled in points
		weights[2] = 1;
	}

	if (thread && extrapolatedPoints > 0 && !path.empty()) {
		thread->drivingPathDebugDrawCircle(path.back(), 10, CV_RGB(0, 255, 0));
	}

	weights[0] = 0; // disregard first point in average
	while ((weights[2] > 0 || weights[4] > 0 || weights[3] > 0) && skipped < 5) {
		// still have left/right lane left
		// TODO fix damn weights
		points[1] = getExtrapolatedPoint(path.size());
		if (thread) {
			thread->drivingPathDebugDrawCircle(points[1], 5, CV_RGB(255, 255, 255));
		}
		weights[1] = weightExtrExtr;
		points[2] = getCorrespPoint(l.rightLane, points[1], true, rightLaneOffset, weights[2]);
		weights[2] *= weightExtrRight;
		points[3] = getCorrespPoint(l.leftLane, points[1], false, leftLaneOffset, weights[3]);
		weights[3] *= weightExtrLeft;
		points[4] = getCorrespPoint(l.centerLane, points[1], false, centerLaneOffset, weights[4]);
		weights[4] *= weightExtrCenter;
		Point2d newP = getAvgPoint(points, weights);
		if (pointBehind(path.size(), newP) || !pointContinuesLane(path, newP)) {
			if (logger) {
				logger->log() << "abort extrapolation" << endl;
			}
			break;
		}
		if (logger) {
			logger->log() << "put " << weights[0] << " " << weights[1] << " " << weights[2] << " " << weights[3] << " " << weights[4] << endl;
		}
		appendItem(getAvgPoint(points, weights));
		++extrapolatedPoints;
		if (thread) {
			thread->drivingPathDebugDrawFunc(points, weights);
		}
	}
	if (logger) {
		logger->log() << "Processing LaneData with " << l.rightLane.size() << " right, " << l.leftLane.size()
					  << " left and " << l.centerLane.size() << " center points. Removed " << removed
					  << " points behind current pose, removed " << removed2 << " points during adjustments, updated "
					  << adjusted << " points from lanedata, extrapolated " << extrapolatedPoints << " and skipped " << skipped << " generated points. Have " << path.size() << " points total." << endl;
	}
	// TODO (smooth) and resample
}

Point2d DrivingPath::getExtrapolatedPoint(size_t idx) const
{
	assert(path.size() > 1 && idx <= path.size());
	Point2d dummy1 = path[0];
	Point2d dummy2 = path[1] - path[0];
	return extrapolateLine(path, idx, stepSize, dummy1, dummy2);
}

bool DrivingPath::pointBehind(size_t i, const Point2d &p)
{
	if (i <= 1 || i > path.size()) {
		return false;
	}
	Point2d a = path[i - 2];
	Point2d b = path[i - 1];
	return orthoProject(p, a, b) < norm(b - a);
}

void DrivingPath::eraseItem(size_t i)
{
	assert(i >= 0 && i < path.size());
	path.erase(path.begin() + i);
	pathWeights.erase(pathWeights.begin() + i);
}

void DrivingPath::appendItem(const Point2d &p)
{
	path.push_back(p);
	pathWeights.push_back(initialPointWeight);
}

void DrivingPath::updateItem(size_t i, const Point2d &p)
{
	path[i] = p;
	if (ageBasedWeighting) {
		pathWeights[i] += weightIncrement;
	}
}

int DrivingPath::eraseOldPoints(const Point2d &pose)
{
	int removed = 0;
	for (size_t i = 0; i < path.size() - 1; ++i) {
		if (orthoProject(pose, path[i], path[i + 1]) <= rejectionDistance && norm(pose - path[i + 1]) <= rejectionDistance) {
			break;
		}
		eraseItem(0);
		++removed;
		--i;
	}
	if (path.size() == 1 && norm(pose - path[0]) > stepSize) {
		eraseItem(0);
		++removed;
	}
	return removed;
}

int DrivingPath::fillInLane(const LaneData &l, int lane, DrivingPathDebugDrawer *thread, SR_Logger *logger)
{
	const vector<Point2d> *activeLane = NULL;
	if (lane == 2) {
		activeLane = &l.rightLane;
	} else if (lane == 3) {
		activeLane = &l.leftLane;
	} else if (lane == 4) {
		activeLane = &l.centerLane;
	} else {
		return 0;
	}
	int filledIn = 0;
	if (!activeLane->empty() && !path.empty() && !pointBehind(path.size(), activeLane->front())) {
		if (logger) {
			logger->log() << "Lane begins after path ends, distance " << norm(path.back() - activeLane->front()) << endl;
		}
		float weight = 0;
		vector<Point2d> points;
		vector<float> weights;
		points.resize(5);
		weights.resize(5);
		Point2d pc = getCorrespPoint(*activeLane, activeLane->front(), lane == 2, lane == 3 ? leftLaneOffset : (lane == 2 ? rightLaneOffset : centerLaneOffset), weight);
		Point2d nn = pc - path.back();
		double dist = norm(nn);
		Point2d px = path.back();
		nn /= dist;
		weights[0] = 0;
		weights[1] = initialPointWeight;
		weights[lane] = 1 * initialPointWeight;
		int steps = dist / stepSize + 1;
		if (steps > 10) {
			if (logger) {
				logger->log() << "Trying to fill in " << steps << " steps!" << endl;
			}
		}
		for (int i = 0; i < steps && i < 10; ++ i) {
			++filledIn;
			points[1] = getExtrapolatedPoint(path.size());
			points[lane] = px + i * stepSize * nn;
			if (lane != 2) {
				points[2] = getCorrespPoint(l.rightLane, points[lane], true, rightLaneOffset, weights[2]);
			}
			if (lane != 3) {
				points[3] = getCorrespPoint(l.leftLane, points[lane], false, leftLaneOffset, weights[3]);
			}
			if (lane != 4) {
				points[4] = getCorrespPoint(l.centerLane, points[lane], false, centerLaneOffset, weights[4]);
			}
			if (thread) {
				thread->drivingPathDebugDrawFunc(points, weights);
			}
			Point2d avg = getAvgPoint(points, weights);
			if (!pointBehind(path.size(), avg)) {
				appendItem(avg);
			} else if (logger) {
				logger->log() << "Fill in: Skipping boint as it's behind line" << endl;
			}
		}
	}
	return filledIn;
}

float DrivingPath::getWeightExtrCenter() const
{
	return weightExtrCenter;
}

void DrivingPath::setWeightExtrCenter(float value)
{
	weightExtrCenter = value;
}

float DrivingPath::getWeightExtrLeft() const
{
	return weightExtrLeft;
}

void DrivingPath::setWeightExtrLeft(float value)
{
	weightExtrLeft = value;
}

float DrivingPath::getWeightExtrRight() const
{
	return weightExtrRight;
}

void DrivingPath::setWeightExtrRight(float value)
{
	weightExtrRight = value;
}

float DrivingPath::getWeightExtrExtr() const
{
	return weightExtrExtr;
}

void DrivingPath::setWeightExtrExtr(float value)
{
	weightExtrExtr = value;
}

float DrivingPath::getWeightUpdCenter() const
{
	return weightUpdCenter;
}

void DrivingPath::setWeightUpdCenter(float value)
{
	weightUpdCenter = value;
}

float DrivingPath::getWeightUpdLeft() const
{
	return weightUpdLeft;
}

void DrivingPath::setWeightUpdLeft(float value)
{
	weightUpdLeft = value;
}

float DrivingPath::getWeightUpdRight() const
{
	return weightUpdRight;
}

void DrivingPath::setWeightUpdRight(float value)
{
	weightUpdRight = value;
}

float DrivingPath::getWeightUpdExtr() const
{
	return weightUpdExtr;
}

void DrivingPath::setWeightUpdExtr(float value)
{
	weightUpdExtr = value;
}

int DrivingPath::getNumPoints() const
{
	return numPoints;
}

void DrivingPath::setNumPoints(int value)
{
	if (value <= 0) {
		value = 5;
	}
	numPoints = value;
}

double DrivingPath::getStepSize() const
{
	return stepSize;
}

double DrivingPath::getRejectionDistance() const
{
	return rejectionDistance;
}

float DrivingPath::getWeightIncrement() const
{
	return weightIncrement;
}

float DrivingPath::getInitialPointWeight() const
{
	return initialPointWeight;
}

bool DrivingPath::getAgeBasedWeighting() const
{
	return ageBasedWeighting;
}

void DrivingPath::setRejectionDistance(double value)
{
	rejectionDistance = value;
}

void DrivingPath::setWeightIncrement(float value)
{
	weightIncrement = value;
}

void DrivingPath::setInitialPointWeight(float value)
{
	initialPointWeight = value;
}

void DrivingPath::setAgeBasedWeighting(bool value)
{
	ageBasedWeighting = value;
}

void DrivingPath::setStepSize(double value)
{
	stepSize = value;
}

const vector<Point2d> *DrivingPath::getPath() const
{
	return &path;
}

OdometryPose DrivingPath::getPoseFromIdx(const OdometryPose &currPose, size_t idx) const
{
	OdometryPose p;
	if (idx < 0 || idx >= path.size()) {
		p = currPose;
		p.speed = 0;
		return p;
	}

	p.x = path[max(idx, path.size() - 1)].x;
	p.y = path[max(idx, path.size() - 1)].y;
	Point2d vec = getOdometryDirection(currPose);
	if (idx < path.size() - 2) {
		vec = path[idx + 1] - path[idx];
	} else if (idx > 0) {
		vec = path[idx] - path[idx - 1];
	}
	p.theta = getOrientationAngle(vec);
	return p;
}

OdometryPose DrivingPath::getInterpPose(size_t i, double dist) const
{
	assert(i >= 0 && i < path.size() - 1);
	Point2d p = path[i];
	Point2d n = path[i + 1] - path[i];
	n /= norm(n);
	p += n * dist;
	OdometryPose pp;
	pp.x = p.x;
	pp.y = p.y;
	pp.theta = getOrientationAngle(n);
	pp.speed = 1.0;
	return pp;
}

void DrivingPath::resetPath(const OdometryPose &currPose)
{
	path.clear();
	pathWeights.clear();
	Point2d p(currPose.x, currPose.y);
	Point2d dir = getOdometryDirection(currPose);
	for (size_t i = 0; i < numPoints; ++i) {
		path.push_back(p + i * stepSize * dir);
		pathWeights.push_back(initialPointWeight);
	}
}

bool DrivingPath::empty() const
{
	return path.empty();
}

DrivingPathDebugDrawer::~DrivingPathDebugDrawer(){}
