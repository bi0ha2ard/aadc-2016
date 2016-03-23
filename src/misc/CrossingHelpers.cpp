#include "CrossingHelpers.h"
#include "FastRot.h"
#include "OdometryHelpers.h"

static const double backOffs = 1.0;
static const double laneOffs = .22;
static const double fwdOffs = .60;

ControllerCommand getCrossingExit(const CrossingDetectionEvent &e, uint8_t exit)
{
	if (exit == EXIT_STRAIGHT) {
		return getCrossingExit(e, ManeuverStraight);
	} else if (exit == EXIT_LEFT) {
		return getCrossingExit(e, ManeuverLeft);
	} else if (exit == EXIT_RIGHT) {
		return getCrossingExit(e, ManeuverRight);
	}
	return getCrossingExit(e, ManeuverStraight);
}

ControllerCommand getCrossingStop(const CrossingDetectionEvent &e, double offs)
{
	Point2d poseBase = e.center - (backOffs + offs) * e.orientationVector + laneOffs * rotatePoint<270>(e.orientationVector);
	ControllerCommand c;
	c.cruiseVelocity = .7;
	c.type = DesiredPose;
	c.pose.x = poseBase.x;
	c.pose.y = poseBase.y;
	c.pose.speed = 0;
	c.pose.theta = getOrientationAngle(e.orientationVector);
	return c;
}

ControllerCommand getCrossingExit(const CrossingDetectionEvent &e, ManeuverType m, double extraOffset, double extraOffset2)
{
	Point2d poseBase = e.center;
	Point2d poseNormal = e.orientationVector;
	Point2d rightN = rotatePoint<270>(poseNormal);
	ControllerCommand c;
	c.type=DesiredPose;
	if (m == ManeuverStraight) {
		poseBase += rightN * laneOffs + (fwdOffs + extraOffset) * poseNormal;
		// poseNormal = poseNormal;
		c.type = DesiredPose;
	} else if (m == ManeuverLeft) {
		poseBase = poseBase - rightN * (fwdOffs + extraOffset) + poseNormal * (laneOffs + extraOffset2);
		poseNormal = -rightN;
		c.type = DesiredCrossing;
	} else if (m == ManeuverRight) {
		// Make right pose right further on
		poseBase = poseBase + rightN * (fwdOffs + extraOffset) - poseNormal * (laneOffs + extraOffset2);
		poseNormal = rightN;
		c.type = DesiredCrossing;
	}
	c.cruiseVelocity = .5;
	c.pose.x = poseBase.x;
	c.pose.y = poseBase.y;
	c.pose.speed = .5;
	c.pose.theta = getOrientationAngle(poseNormal);
	return c;
}

void mergeCrossings(std::vector<CrossingDetectionEvent> &merged, CrossingDetectionEvent e, double distThreshold)
{
	for (size_t i = 0; i < merged.size(); ++i) {
		if (norm(e.center - merged[i].center) < distThreshold) {
			merged[i].center = merged[i].center * merged[i].numDetections + e.center * e.numDetections;
			merged[i].orientationVector = merged[i].orientationVector * merged[i].numDetections + e.orientationVector * e.numDetections;
			merged[i].numDetections += e.numDetections;
			merged[i].center /= merged[i].numDetections;
			merged[i].orientationVector /= norm(merged[i].orientationVector);
			merged[i].exits |= e.exits;
			merged[i].stopLine |= e.stopLine;
			return;
		}
	}
	// Not found
	merged.push_back(e);
}
