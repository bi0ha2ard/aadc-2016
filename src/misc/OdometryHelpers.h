#ifndef SR_ODOMETRY_HELPERS_H_
#define SR_ODOMETRY_HELPERS_H_

#include <odometry_data.h>
#include <opencv/cv.h>
#include "FastRot.h"

cv::Point2d normalToWorld(cv::Point2d n, const OdometryPose &p);

double getOrientationAngle(const cv::Point2d &n);

Point2d getOdometryDirection(const OdometryPose &p);
Point2d getOdometryPos(const OdometryPose &p);

#endif
