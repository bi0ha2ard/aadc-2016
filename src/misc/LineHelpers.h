#ifndef SR_LINEHELPERS_H_
#define SR_LINEHELPERS_H_

#include <opencv/cv.h>
#include <typeinfo>

using namespace std;
using namespace cv;

typedef vector<Point2i> Line2i;
typedef vector<Point2d> Line2d;

double fixAngle(double a);
double fixAngleDeg(double a);


/**
 * @brief getExtrapolatedPoint extrapolate a line
 * @param line line to extrapolate
 * @param pos index of extrapolated point in the line, <= line.size()
 * @param extrapolationDist
 * @param defaultPoint if line is empty, this point is returned
 * @param defaultDirection if line is 1 long, this direction is used to extrapolate
 * @return the extrapolated point
 */
template <typename T> Point2d extrapolateLine(const vector<Point_<T> > &line, size_t pos, double extrapolationDist, const Point2d &defaultPoint, const Point2d &defaultDirection, size_t backsteps = 12)
{
	assert(pos <= line.size());
	bool distChecksEnabled = (typeid(T) == typeid(int));
	if (line.empty() || pos == 0) {
		// Bad estimate based on no info
		return defaultPoint;
	}
	if (line.size() == 1 || pos == 1) {
		// point perp to search dir starting from last known line point
		return Point2d(line[0]) + defaultDirection * extrapolationDist / norm(defaultDirection);
	}
	if (line.size() == 2 || pos == 2) {
		// linear extrapolation
		Point2d vec = line[1] - line[0];
		return Point2d(line[1]) + extrapolationDist * vec / norm(vec);
	}

	// make weighted average of angle rate and extrapolate
	double angleRate = 0;
	double currWeight = 1;
	double count = 0;
	ssize_t start = pos - backsteps;
	if (start < 1) {
		start = 1;
	}
	Point2d currStep = line[start] - line[start - 1];
	double lastAngle = atan2(currStep.y, currStep.x);
	double currAngle = 0;
	for (size_t i = start; i < pos - 1;) {
		size_t j = i;
		do {
			++j;
			currStep = line[j] - line[i];
		} while (distChecksEnabled && norm(currStep) < 8 && j < line.size() - 1);
		currAngle = atan2(currStep.y, currStep.x);
		double anglediff = fixAngle(currAngle - lastAngle);
		/*
		if (anglediff > m_pi_4 / 2) {
			// warning
		}
		*/
		angleRate += currWeight * anglediff;
		i = j;
		count += currWeight;
		//currWeight += .25;
		lastAngle = currAngle;
	}
	double nextAngle = currAngle + (count == 0 ? angleRate : angleRate / count);
	return Point2d(line[pos - 1]) + Point2d(cos(nextAngle) * extrapolationDist, sin(nextAngle) * extrapolationDist);
}

/**
 * Return a positive value if the points a, b, and c occur
 * in counterclockwise order; a negative value if they occur
 * in clockwise order; and zero if they are collinear.  The
 * result is also a rough approximation of twice the signed
 * area of the triangle defined by the three points.
 * @param a
 * @param b
 * @param c
 * @return > 0 if a b c are counterclockwise, < 0 if clockwise and 0 if colinear
 */
template <typename T>
static T getOrientation(const cv::Point_<T> &a, const cv::Point_<T> &b, const cv::Point_<T> &c)
{
	cv::Point_<T> ac = a - c;
	cv::Point_<T> bc = b - c;
	return ac.x * bc.y - ac.y * bc.x;
}

double getLineFitError(const std::vector<cv::Point2i> &points, const cv::Vec4f &line);
double getLinearityError(const std::vector<cv::Point2i> &points, ssize_t start, ssize_t end);

int distSquare(const cv::Point &a, const cv::Point &b);
double distSquareD(const cv::Point2d &a, const cv::Point2d &b);
int distSquare(const cv::Point &a);
double angle360(const cv::Point &p);
bool isParallel(double angle1, double angle2, double eta = .1);
double pointDistToLine(const cv::Point &p, const cv::Vec4f &line);

/**
 * @brief getCorrespPoint returns a point on the line perpendicular to l through p
 * at distance distance, either to the left or the right
 * @param l the line
 * @param p the point
 * @param toLeft return point to left or right of line
 * @param distance distance point <-> line
 * @param weight is set to 0 if no point is found
 * @param closeLimit if under this distance, use normal of line instead of
 * line connecting point and linepoint for the point generation
 * @return the corresponding point or p if none found
 */
Point2d getCorrespPoint(const Line2d &l, const Point2d &p, bool toLeft, double distance, float &weight, double closeLimit = .01);

template <typename T> Point_<T> getAvgPoint(const Point_<T> &a, const Point_<T> &b)
{
	return a + (b - a) / 2.0;
}

template <typename T> Point2d getAvgPoint(const vector<Point_<T> > &points, const vector<float> &weights)
{
	assert(points.size() == weights.size());
	assert(!points.empty());
	double xs = 0;
	double ys = 0;
	double sum = 0;
	for (size_t i = 0; i < points.size(); ++i) {
		xs += points[i].x * weights[i];
		ys += points[i].y * weights[i];
		sum += weights[i];
	}
	return Point_<T>(xs / sum, ys / sum);
}

/**
 * @brief orthoProject orthogonal projection
 * @param p point
 * @param a line AB
 * @param b line AB
 * @return distance along AB
 */
double orthoProject(const Point2d &p, const Point2d &a, const Point2d &b);
bool pointInfrontOf(const Point2d &p, const Point2d &a, const Point2d &dir);

Point2d getDirection(const Line2d &l, size_t idx);

bool linesEqual(const vector<Point2i> &a, const vector<Point2i> &b);
bool linesEquald(const Line2d &a, const Line2d &b, double EPSILON = 0.01);
bool pointsEqual(const Point2d &a, const Point2d &b, double EPSILON = 0.01);

template <typename T> bool pointToSideOf(const vector<Point_<T> > &line, const Point_<T> &p, bool left, double *dist)
{
	if (line.size() < 2) {
		if (dist) {
			*dist = -1;
		}
		return false;
	}
	int toSide = false;
	double closestDist = INT64_MAX;
	for (size_t i = 0; i < line.size() - 1; ++i) {
		double dist = norm(line[i + 1] - p);
		if (dist < closestDist) {
			closestDist = dist;
			if (left) {
				toSide = getOrientation(line[i], line[i + 1], p) <= 0;
			} else {
				toSide = getOrientation(line[i], line[i + 1], p) >= 0;
			}
		}
	}
	if (dist) {
		*dist = closestDist;
	}
	return toSide;
}

/**
 * @brief checkAngle Note that for Point2i results will be bad if p is very close to the end of lane
 * @param lane
 * @param p
 * @param pos the place to check against
 * @param threshold
 * @return
 */
template <typename T>
bool pointContinuesLane(const std::vector<cv::Point_<T> > &lane, const cv::Point2d &p, ssize_t pos = -1, double threshold = M_PI_4 / 2.0)
{
	if (pos < 0) {
		pos = lane.size() - 1;
	}
	if (pos < 1) {
		//LOG_INFO("Size too small!");
		return true;
	}
	assert(pos < lane.size());
	Point2d currDir = p - Point2d(lane[pos]);
	/*
	if (typeid (T) == typeid(int) && norm(currDir) < 8) {
		// if int, short stuff is bad.
		return false;
	}
	*/
	double currAngle = atan2(currDir.y, currDir.x);
	currDir = lane[pos] - lane[pos - 1];
	double angleDiff = fixAngle(currAngle - atan2(currDir.y, currDir.x));
	return fabs(angleDiff) < threshold;
}

void drawFadedLine(Mat &m, Line2i l, const Scalar &c1, const Scalar &c2);

#endif
