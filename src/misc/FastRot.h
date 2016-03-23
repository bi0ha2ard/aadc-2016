#ifndef SR_FASTROT_H_
#define SR_FASTROT_H_

#include <opencv/cv.h>
using namespace cv;

#define COS45DEG 0.70710678118654752440

// Fast rotations for common angles using templates

template <int R> inline Point2d rotatePoint(const Point2d &p)
{
	double sina = sin(R * M_PI / 180.0);
	double cosa = cos(R * M_PI / 180.0);
	return Point2d(cosa * p.x - sina * p.y, sina * p.x + cosa * p.y);
}

template <> inline Point2d rotatePoint<0>(const Point2d &p)
{
	return p;
}

template <> inline Point2d rotatePoint<180>(const Point2d &p)
{
	return -p;
}

template <> inline Point2d rotatePoint<90>(const Point2d &p)
{
	return Point2d(-p.y, p.x);
}

template <> inline Point2d rotatePoint<270>(const Point2d &p)
{
	return Point2d(p.y, -p.x);
}

template <> inline Point2d rotatePoint<45>(const Point2d &p)
{
	return Point2d( COS45DEG * p.x - COS45DEG * p.y,  COS45DEG * p.x + COS45DEG * p.y);
}

template <> inline Point2d rotatePoint<135>(const Point2d &p)
{
	return Point2d(-COS45DEG * p.x - COS45DEG * p.y,  COS45DEG * p.x - COS45DEG * p.y);
}

template <> inline Point2d rotatePoint<225>(const Point2d &p)
{
	return Point2d(-COS45DEG * p.x + COS45DEG * p.y, -COS45DEG * p.x - COS45DEG * p.y);
}

template <> inline Point2d rotatePoint<315>(const Point2d &p)
{
	return Point2d( COS45DEG * p.x + COS45DEG * p.y, -COS45DEG * p.x + COS45DEG * p.y);
}

inline Point2d rotatePointD(const Point2d &p, double angle)
{
	double sina = sin(angle);
	double cosa = cos(angle);
	return Point2d(cosa * p.x - sina * p.y, sina * p.x + cosa * p.y);
}

inline Point2d rotatePointDDeg(const Point2d &p, double angle)
{
	return rotatePointD(p, angle * M_PI / 180.0);
}

#endif
