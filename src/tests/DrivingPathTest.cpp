#include "DrivingPathTest.h"
#include <QString>
#include <QTest>

#include "../misc/LineHelpers.h"

SR_Logger logger;

QString printLine(const Line2d &l)
{
	QString s;
	for (size_t i = 0; i < l.size(); ++i) {
		s.append(QString("%1: [%2, %3] ").arg(i).arg(l[i].x).arg(l[i].y));
	}
	return s;
}

DrivingPathTest::DrivingPathTest()
{
	dummyPose.x = 0;
	dummyPose.y = 0;
}

void DrivingPathTest::testEmptyPath()
{
	reset();
	Line2d origLine = (*p.getPath());
	p.processLaneData(l, dummyPose);
	QVERIFY2(linesEquald(origLine, *p.getPath()), "Empty line: Lines not equal!");
}

void DrivingPathTest::testExtrapolate()
{
	Line2d origLine = *p.getPath();
	for (size_t i = 0; i < origLine.size(); ++i) {
		reset();
		Point2d ret = p.getExtrapolatedPoint(i);
		QString err = QString("reset, i: %1 [%2, %3], [%4, %5]").arg(i).arg(origLine[i].x).arg(origLine[i].y).arg(ret.x).arg(ret.y);
		QVERIFY2(pointsEqual(ret, origLine[i]), err.toAscii().data());
	}
	reset();
	for (size_t i = 0; i < origLine.size(); ++i) {
		Point2d ret = p.getExtrapolatedPoint(i);
		QString err = QString("no reset, i: %1 [%2, %3], [%4, %5]").arg(i).arg(origLine[i].x).arg(origLine[i].y).arg(ret.x).arg(ret.y);
		QVERIFY2(pointsEqual(ret, origLine[i]), err.toAscii().data());
	}
	reset();
	Point2d pp = p.getExtrapolatedPoint(origLine.size());
	QCOMPARE(pp, Point2d(origLine.back() + Point2d(p.getStepSize(), 0)));
}

void DrivingPathTest::testParallelPath()
{
	reset();
	Line2d origLine = (*p.getPath());
	l.rightLane.push_back(Point2d(0, -.22));
	l.rightLane.push_back(Point2d(.5, -.22));
	l.rightLane.push_back(Point2d(1, -.22));
	l.rightLane.push_back(Point2d(1.5, -.22));
	l.rightLane.push_back(Point2d(2, -.22));
	p.processLaneData(l, dummyPose, NULL, &logger);
	QString res = QString("Parallel right lane: Lines not equal:\n%1\n%2").arg(printLine(origLine)).arg(printLine(*p.getPath()));
	QVERIFY2(linesEquald(origLine, *p.getPath()), res.toAscii().data());
}

/*
void DrivingPathTest::testErase()
{
	reset();
	OdometryPose forwardPose;
	forwardPose.x = 1;
	forwardPose.y = 0;
	p.processLaneData(l, forwardPose);
	QString err = QString("Expected first element x to be >= 1, was %1, size %2").arg(p.getPath()->front().x).arg(p.getPath()->size());
	QVERIFY2(p.getPath()->front().x >= 1, err.toAscii().data());
	reset();
	size_t origSize = p.getPath()->size();
	OdometryPose backwardPose;
	backwardPose.x = -1;
	backwardPose.y = 0;
	p.processLaneData(l, backwardPose);
	QCOMPARE(p.getPath()->size(), origSize);
	reset();
	Point2d testP(10, 0);
	p.eraseOldPoints(testP);
	QCOMPARE(p.empty(), true);
}
*/

void DrivingPathTest::testPointBehind()
{
	reset();
	Point2d point(5, 0);
	QCOMPARE(p.pointBehind(p.getPath()->size(), point), false);
	point = p.getPath()->back();
	QCOMPARE(p.pointBehind(p.getPath()->size(), point), false);
	point.x += 1;
	QCOMPARE(p.pointBehind(p.getPath()->size(), point), false);
	point.x -= 2;
	QCOMPARE(p.pointBehind(p.getPath()->size(), point), true);
	point = Point2d(-1, 0);
	QCOMPARE(p.pointBehind(p.getPath()->size(), point), true);
}

void DrivingPathTest::reset()
{
	p = DrivingPath();
	l = LaneData();
}



ostream &SR_Logger::log()
{
	return cout;
}

QTEST_APPLESS_MAIN(DrivingPathTest)
