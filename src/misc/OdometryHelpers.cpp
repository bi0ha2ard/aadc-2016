#include "OdometryHelpers.h"


Point2d normalToWorld(Point2d n, const OdometryPose &p)
{
	n.y *= -1; // Flip y to get right handed CS
	cv::Point2d nWorld = rotatePointDDeg(n, p.theta + 270);
	return nWorld / norm(nWorld);
}

double getOrientationAngle(const cv::Point2d &n)
{
	double a = atan2(n.y, n.x);
	if (a < 0) {
		a += 2 * M_PI;
	}
	return a * 180.0 / M_PI;
}

Point2d getOdometryDirection(const OdometryPose &p)
{
	double t = p.theta * M_PI / 180.0;
	return Point2d(cos(t), sin(t));
}

Point2d getOdometryPos(const OdometryPose &p)
{
	return Point2d(p.x, p.y);
}
