#ifndef SR_DRIVINGPATH_TEST_H_
#define SR_DRIVINGPATH_TEST_H_

#include <iostream>

class SR_Logger
{
public:
	std::ostream &log();
};

#include <QObject>
#include "../SR_PathPlanner/DrivingPath.h"
#include <map_data.h>

class DrivingPathTest : public QObject
{
	Q_OBJECT

public:
	DrivingPathTest();

private Q_SLOTS:
	void testEmptyPath();
	void testExtrapolate();
	void testParallelPath();
	//void testErase();
	void testPointBehind();

private:
	void reset();
	DrivingPath p;
	LaneData l;
	OdometryPose dummyPose;
};


#endif
