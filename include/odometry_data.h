#ifndef _ODOMETRY_DATA_H_
#define _ODOMETRY_DATA_H_

#include <iostream>

typedef struct {
	double x;
	double y;
	double theta;
	double speed;
} OdometryPose;

enum ControllerCommandType
{
    DesiredPose,
    DesiredCrossing,
    ParkingParallel,
    ParkingCross,
    PullOutParallel,
    PullOutCrossLeft,
    PullOutCrossRight
};
typedef struct {
        OdometryPose pose;
        ControllerCommandType type;
        float cruiseVelocity;
} ControllerCommand;

inline std::ostream &operator <<(std::ostream &os, const ControllerCommand &c)
{
        os << "[" << c.pose.x << ";" << c.pose.y << ";" << c.pose.theta << ";" << c.pose.speed << ";cruise;" << c.cruiseVelocity << ";type;" << c.type << "]";
	return os;
}

inline std::ostream &operator <<(std::ostream &os, const OdometryPose &p)
{
	os << "[" << p.x << ";" << p.y << ";" << p.theta << "]";
	return os;
}

#define MEDIA_TYPE_ODOMETRY 0x00001337
#define MEDIA_SUBTYPE_ODOMETRY_POSE 0x00000001

#define MEDIA_TYPE_CONTROLLERCOMMAND 0x00001338
#define MEDIA_SUBTYPE_CONTROLLER_COMMAND 0x00000001


#endif
