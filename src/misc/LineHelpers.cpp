#include "LineHelpers.h"
#include <opencv2/imgproc.hpp>

double getLineFitError(const std::vector<Point2i> &points, const Vec4f &line)
{
	double sum = 0;
	size_t n = points.size();
	for (size_t i = 0; i < n; ++i) {
		sum += pointDistToLine(points[i], line);
	}
	return sum / n;
}

double getLinearityError(const std::vector<Point2i> &points, ssize_t start, ssize_t end)
{
	if (end - start < 2 || start < 0 || end > static_cast<ssize_t>(points.size()) - 1) {
		return -1;
	}
	vector<Point2i> currPoints(points.begin() + start, points.begin() + end);
	Vec4f line;
	cv::fitLine(currPoints, line, CV_DIST_L2, 0, .01, .01);
	return getLineFitError(currPoints, line);
}

int distSquare(const Point &a, const Point &b)
{
	return (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y -a.y);
}

double distSquareD(const Point2d &a, const Point2d &b)
{
	return (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y -a.y);
}

int distSquare(const Point &a)
{
	return a.x * a.x + a.y * a.y;
}

double angle360(const Point &p)
{
	double angle = atan2(p.y, p.x);
	return angle < 0 ? angle + 2 * M_PI : angle;
}

bool isParallel(double angle1, double angle2, double eta)
{
	double diff = fabs(angle1 - angle2);
	while (diff > 2 * M_PI) {
		diff -= 2 * M_PI;
	}
	return (diff < eta || diff > (2 * M_PI - eta) || fabs(diff - M_PI) < eta);
}

double pointDistToLine(const Point &p, const Vec4f &line)
{
	Point2f a(line[2], line[3]);
	Point2f b(line[0], line[1]);
	b = a + 100 * b;
	double dist = fabs((b.y - a.y) * p.x - (b.x - a.x) * p.y + b.x * a.y - b.y * a.x);
	Point2f along = b - a;
	return dist / norm(along);
}

double fixAngle(double a) {
	return ((a > M_PI) ? (a - 2 * M_PI) : ((a < - M_PI) ? (a + 2 * M_PI) : a));
}

double fixAngleDeg(double a) {
	return ((a > 180) ? (a - 2 * 180) : ((a < - 180) ? (a + 2 * 180) : a));
}

Point2d getCorrespPoint(const Line2d &l, const Point2d &p, bool toLeft, double distance, float &weight, double closeLimit)
{
	if (l.size() < 2) {
		weight = 0;
		return p;
	}
	double minDist = 999;
	ssize_t minIdx = -1;
	for (size_t i = 0; i < l.size(); ++i) {
		double dist;
		if ((dist = norm(p - l[i])) < minDist) {
			minDist = dist;
			minIdx = i;
			if (minDist == 0) {
				break;
			}
		}
	}
	if (minIdx >= 0) {
		if (minDist < closeLimit) {
			Point2d vec = getDirection(l, minIdx);
			Point2d n;
			if (toLeft) {
				n.x = -vec.y;
				n.y = vec.x;
			} else {
				n.x = vec.y;
				n.y = -vec.x;
			}
			weight = 1;
			return l[minIdx] + distance * n;
		}
		double ortho = -1;
		Point2d base;
		Point2d next;
		Point2d s;
		if (minIdx > 0) {
			ortho = orthoProject(p, l[minIdx - 1], l[minIdx]);
			base = l[minIdx - 1];
			next = l[minIdx];
			s = l[minIdx] - l[minIdx - 1];
		}
		if (ortho < 0 && minIdx < static_cast<ssize_t>(l.size()) - 1) {
			ortho = orthoProject(p, l[minIdx], l[minIdx + 1]);
			base = l[minIdx];
			next = l[minIdx + 1];
			s = l[minIdx + 1] - l[minIdx];
		}
		if (ortho < 0 || ortho > 1.5 * norm(s)) {
			// extrapolated backwards or too far forward
			weight = 0;
			return p;
		}
		s /= norm(s);
		Point2d pp = base + ortho * s;
		Point2d n = p - pp;
		n /= norm(n);
		bool onLeft = getOrientation(p, base, next) > 0;
		if (onLeft != toLeft) {
			n *= -1;
		}
		// TODO set weight
		weight = 1;
		return pp + n * distance;
	}
	weight = 0;
	return p;
}

double orthoProject(const Point2d &p, const Point2d &a, const Point2d &b)
{
	Point2d v = p - a;
	Point2d s = b - a;
	double n = norm(s);
	if (n == 0) {
		return -1;
	}
	s /= n;
	return v.dot(s);
}

Point2d getDirection(const Line2d &l, size_t idx)
{
	Point2d vec(1, 0);
	if (idx < l.size() - 1) {
		vec = l[idx + 1] - l[idx];
	} else if (idx > 0) {
		vec = l[idx] - l[idx - 1];
	}
	return vec / norm(vec);
}

bool linesEquald(const Line2d &a, const Line2d &b, double EPSILON)
{
	if (a.size() != b.size()) {
		return false;
	}
	for (size_t i = 0; i < a.size(); ++i) {
		if (fabs(a[i].x - b[i].x) > EPSILON || fabs(a[i].y - b[i].y) > EPSILON) {
			return false;
		}
	}
	return true;
}

bool linesEqual(const vector<Point2i> &a, const vector<Point2i> &b)
{
	if (a.size() != b.size()) {
		return false;
	}
	for (size_t i = 0; i < a.size(); ++i) {
		if (a[i] != b[i]) {
			return false;
		}
	}
	return true;
}

bool pointsEqual(const Point2d &a, const Point2d &b, double EPSILON)
{
	return fabs(a.x - b.x) < EPSILON && fabs(a.y - b.y) < EPSILON;
}

void drawFadedLine(Mat &m, Line2i l, const Scalar &c1, const Scalar &c2)
{
	double n = l.size();
	Scalar diff = c2 - c1;
	for (size_t i = 1; i < n; ++i) {
		cv::line(m, l[i - 1], l[i], c1 + diff * static_cast<double>(i) / n, 1, CV_AA);
	}
}

bool pointInfrontOf(const Point2d &p, const Point2d &a, const Point2d &dir)
{
	return orthoProject(p, a, a + dir) > 0;
}
