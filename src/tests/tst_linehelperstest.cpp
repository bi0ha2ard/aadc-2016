#include "tst_linehelperstest.h"
#include <QString>
#include <QtTest>

#include "../misc/LineHelpers.h"
#include "../misc/FastRot.h"

LineHelpersTest::LineHelpersTest()
{
}

void LineHelpersTest::initTestCase()
{
	straightLine.push_back(Point2d(0, 0));
	straightLine.push_back(Point2d(1, 0));
	straightLine.push_back(Point2d(2, 0));
}

void LineHelpersTest::testCorrespPointLeftToLeft()
{
	float weight = -1;
	Point2d res = getCorrespPoint(straightLine, Point2d(1, 1), true, 1, weight, 0.01);
	double dist = norm(res - Point2d(1, 1));
	QString err = getErrStr(Point2d(1, 1), res, 1, weight);
	QVERIFY2((weight != 0 && dist < .1), err.toAscii().data());
}

void LineHelpersTest::testCorrespPointLeftToRight()
{
	float weight = -1;
	Point2d res = getCorrespPoint(straightLine, Point2d(1, 1), false, 1, weight, 0.01);
	double dist = norm(res - Point2d(1, -1));
	QString err = getErrStr(Point2d(1, -1), res, 1, weight);
	QVERIFY2((weight != 0 && dist < .1), err.toAscii().data());
}

void LineHelpersTest::testCorrespPointMiddleToLeft()
{
	float weight = -1;
	Point2d res = getCorrespPoint(straightLine, Point2d(1, 0), true, 1, weight, 0.01);
	double dist = norm(res - Point2d(1, 1));
	QString err = getErrStr(Point2d(1, 1), res, 1, weight);
	QVERIFY2((weight != 0 && dist < .1), err.toAscii().data());
}

void LineHelpersTest::testCorrespPointRightToLeft()
{
	float weight = -1;
	Point2d res = getCorrespPoint(straightLine, Point2d(1, -1), true, 1, weight, 0.01);
	double dist = norm(res - Point2d(1, 1));
	QString err = getErrStr(Point2d(1, 1), res, 1, weight);
	QVERIFY2((weight != 0 && dist < .1), err.toAscii().data());
}

void LineHelpersTest::testCorrespPointRightToRight()
{
	float weight = -1;
	Point2d res = getCorrespPoint(straightLine, Point2d(1, -1), false, 1, weight, 0.01);
	double dist = norm(res - Point2d(1, -1));
	QString err = getErrStr(Point2d(1, -1), res, 1, weight);
	QVERIFY2((weight != 0 && dist < .1), err.toAscii().data());
}

void LineHelpersTest::testPointLeftMiddleLeft()
{
	float weight = -1;
	Point2d res = getCorrespPoint(straightLine, Point2d(.5, 1), true, 1, weight, 0.01);
	double dist = norm(res - Point2d(.5, 1));
	QString err = getErrStr(Point2d(.5, 1), res, 1, weight);
	QVERIFY2((weight != 0 && dist < .1), err.toAscii().data());
}

void LineHelpersTest::testCorrespOffsetLine()
{
	Line2d test;
	test.push_back(Point2d(0, -.22));
	test.push_back(Point2d(1, -.22));
	test.push_back(Point2d(2, -.22));
	test.push_back(Point2d(3, -.22));
	test.push_back(Point2d(4, -.22));
	test.push_back(Point2d(5, -.22));
	test.push_back(Point2d(6, -.22));
	float weight = -1;
	Point2d exp(0, 0);
	Point2d res = getCorrespPoint(test, exp, true, .22, weight, 0.01);
	double dist = norm(res - exp);
	QString err = getErrStr(exp, res, 1, weight);
	QVERIFY2((weight != 0 && dist < .01), err.toAscii().data());

	exp = Point2d(.9, 0);
	res = getCorrespPoint(test, exp, true, .22, weight, 0.01);
	dist = norm(res - exp);
	err = getErrStr(exp, res, 1, weight);
	QVERIFY2((weight != 0 && dist < .01), err.toAscii().data());

	exp = Point2d(1.9, 0);
	res = getCorrespPoint(test, exp, true, .22, weight, 0.01);
	dist = norm(res - exp);
	err = getErrStr(exp, res, 1, weight);
	QVERIFY2((weight != 0 && dist < .01), err.toAscii().data());

	exp = Point2d(2.9, 0);
	res = getCorrespPoint(test, exp, true, .22, weight, 0.01);
	dist = norm(res - exp);
	err = getErrStr(exp, res, 1, weight);
	QVERIFY2((weight != 0 && dist < .01), err.toAscii().data());

	exp = Point2d(2.1, 0);
	res = getCorrespPoint(test, exp, true, .22, weight, 0.01);
	dist = norm(res - exp);
	err = getErrStr(exp, res, 1, weight);
	QVERIFY2((weight != 0 && dist < .01), err.toAscii().data());
}

void LineHelpersTest::testEmptyLine()
{
	Line2d foo;
	Point2d p(1, 1);
	float weight = -1;
	Point2d res = getCorrespPoint(foo, p, false, 1, weight, 0.01);
	QString err = getErrStr(p, res, 0, weight);
	QVERIFY2(weight == 0 && norm(p - res) <= .1, err.toAscii().data());
}

void LineHelpersTest::testSingle()
{
	Line2d foo;
	foo.push_back(Point2d(0, 0));
	Point2d p(1, 1);
	float weight = -1;
	Point2d res = getCorrespPoint(foo, p, false, 1, weight, 0.01);
	QString err = getErrStr(p, res, 0, weight);
	QVERIFY2(weight == 0 && norm(p - res) <= .1, err.toAscii().data());
}

void LineHelpersTest::testFarAhead()
{
	float weight = -1;
	Point2d res = getCorrespPoint(straightLine, Point2d(10, -1), false, 1, weight, 0.01);
	norm(res - Point2d(10, -1));
	QString err = getErrStr(Point2d(10, -1), res, 0, weight);
	QVERIFY2(weight == 0, err.toAscii().data());
}

void LineHelpersTest::testFarBehind()
{
	float weight = -1;
	Point2d res = getCorrespPoint(straightLine, Point2d(-10, -1), false, 1, weight, 0.01);
	norm(res - Point2d(-10, -1));
	QString err = getErrStr(Point2d(-10, -1), res, 0, weight);
	QVERIFY2(weight == 0, err.toAscii().data());
}

void LineHelpersTest::testFirstShortest()
{
	float weight = -1;
	Point2d res = getCorrespPoint(straightLine, Point2d(0, 1), true, 1, weight, 0.01);
	double dist = norm(res - Point2d(0, 1));
	QString err = getErrStr(Point2d(0, 1), res, 1, weight);
	QVERIFY2((weight != 0 && dist < .1), err.toAscii().data());
}

void LineHelpersTest::testLastShortest()
{
	float weight = -1;
	Point2d res = getCorrespPoint(straightLine, Point2d(2, 1), true, 1, weight, 0.01);
	double dist = norm(res - Point2d(2, 1));
	QString err = getErrStr(Point2d(2, 1), res, 1, weight);
	QVERIFY2((weight != 0 && dist < .1), err.toAscii().data());
}

void LineHelpersTest::testExtrapolate()
{
	Line2d l;
	Point2d defaultPoint(0,0);
	Point2d defaultDir(1,0);
	Point2d res = extrapolateLine(l, 0, 1, defaultPoint, defaultDir);
	QVERIFY2(res == defaultPoint, "Empty test failed");
	l.push_back(defaultPoint);
	res = extrapolateLine(l, 1, 1, defaultPoint, defaultDir);
	QVERIFY2(res == defaultPoint + defaultDir, "Single point failed");
	l.push_back(defaultDir);
	res = extrapolateLine(l, 2, 1, defaultPoint, defaultDir);
	QVERIFY2(res == defaultPoint + Point2d(2, 0), "Two points failed");
	l.push_back(Point2d(2, 0));
	res = extrapolateLine(l, 3, 1, defaultPoint, defaultDir);
	QVERIFY2(res == Point2d(3, 0), "Three points failed");
	res = extrapolateLine(l, 0, 1, defaultPoint, defaultDir);
	QVERIFY2(res == defaultPoint, "Empty test 2 failed");
	res = extrapolateLine(l, 1, 1, defaultPoint, defaultDir);
	QVERIFY2(res == defaultPoint + defaultDir, "Single point 2 failed");
	res = extrapolateLine(l, 2, 1, defaultPoint, defaultDir);
	QVERIFY2(res == defaultPoint + Point2d(2, 0), "Two points 2 failed");
	l.push_back(Point2d(4, 0));
	res = extrapolateLine(l, 3, 1, defaultPoint, defaultDir);
	QVERIFY2(res == Point2d(3, 0), "Three points 2 failed");
}

void LineHelpersTest::testOrtho()
{
	double eps = 0.001;
	Point2d testP(0, 1);
	Point2d a(0, 0);
	Point2d b(2, 0);
	double res = orthoProject(testP, a, b);
	QString err = QString("%1, %2").arg(testP.x).arg(res);
	QVERIFY2(fabs(res) < eps, err.toAscii().data());
	testP.x = 1;
	res = orthoProject(testP, a, b);
	err = QString("%1, %2").arg(testP.x).arg(res);
	QVERIFY2(fabs(res - testP.x) < eps, err.toAscii().data());
	testP.x = .5;
	res = orthoProject(testP, a, b);
	err = QString("%1, %2").arg(testP.x).arg(res);
	QVERIFY2(fabs(res - testP.x) < eps, err.toAscii().data());
	testP.x = 2;
	res = orthoProject(testP, a, b);
	err = QString("%1, %2").arg(testP.x).arg(res);
	QVERIFY2(fabs(res - testP.x) < eps, err.toAscii().data());
	testP.x = -1;
	res = orthoProject(testP, a, b);
	err = QString("%1, %2").arg(testP.x).arg(res);
	QVERIFY2(fabs(res - testP.x) < eps, err.toAscii().data());
	err = QString("Was %1").arg(orthoProject(b, a, b));
	testP = Point2d(1, 0);
	QVERIFY2(orthoProject(testP, a, b) == 1.0, err.toAscii().data());
	QVERIFY(orthoProject(a, a, b) == 0);
}

void LineHelpersTest::testAngleCheck()
{
	Point2d p(3, 0);
	QCOMPARE(pointContinuesLane(straightLine, p), true);;
	p.x = 2;
	p.y = 1;
	QCOMPARE(pointContinuesLane(straightLine, p), false);;
	p.y = -1;
	QCOMPARE(pointContinuesLane(straightLine, p), false);;
	Point2d temp(cos(M_PI_4 / 2), sin(M_PI_4 / 2));
	p = straightLine.back() + temp;
	p.x -= .001;
	QCOMPARE(pointContinuesLane(straightLine, p), false);;
	p.x += .002;
	QCOMPARE(pointContinuesLane(straightLine, p), true);;
	p.y *= -1;
	QCOMPARE(pointContinuesLane(straightLine, p), true);;
	p.x -= .002;
	QCOMPARE(pointContinuesLane(straightLine, p), false);;
	QCOMPARE(pointContinuesLane(straightLine, p, -1, M_PI_4), true);
	QCOMPARE(pointContinuesLane(straightLine, p, 1), true);
	Line2d test;
	test.push_back(Point2d(0, -.22));
	test.push_back(Point2d(1, -.22));
	test.push_back(Point2d(2, -.22));
	test.push_back(Point2d(3, -.22));
	test.push_back(Point2d(4, -.22));
	test.push_back(Point2d(5, -.22));
	test.push_back(Point2d(6, -.22));
	p  = Point2d(6, -.22);
	QCOMPARE(pointContinuesLane(test, p, 5), true);
	p = Point2d(1, 0);
	// antiparallel
	QCOMPARE(pointContinuesLane(straightLine, p), false);
	for (size_t i = 1; i < test.size(); ++i) {
		QCOMPARE(pointContinuesLane(test, test[i], i - 1, M_PI_4 / 2.0), true);
	}
}

void LineHelpersTest::testPointInfrontof()
{
	Point2d a(1,0);
	Point2d n(1, 0);
	Point2d x(2, 0);
	QVERIFY(pointInfrontOf(x, a, n));
	QVERIFY(!pointInfrontOf(x, a, -n));
	x.x = 0;
	QVERIFY(!pointInfrontOf(x, a, n));
	QVERIFY(pointInfrontOf(x, a, -n));
	QVERIFY(!pointInfrontOf(a, a, n));
}

void LineHelpersTest::testPointDistToLine()
{
	Vec4f line(1, 0, 0, 0);
	Point2d p(0, 0);
	QCOMPARE(pointDistToLine(p, line), 0.0);
	p.x = -1;
	QCOMPARE(pointDistToLine(p, line), 0.0);
	p.x = 100;
	QCOMPARE(pointDistToLine(p, line), 0.0);
	p.x = 0;
	p.y = 1;
	QCOMPARE(pointDistToLine(p, line), 1.0);
	p.y = -1;
	QCOMPARE(pointDistToLine(p, line), 1.0);
	p.y = 5;
	QCOMPARE(pointDistToLine(p, line), 5.0);
	p.x = 10;
	QCOMPARE(pointDistToLine(p, line), 5.0);
}

void LineHelpersTest::testFastRot()
{
	Point2d p(1, 0);
	QCOMPARE(rotatePoint<0>(p), p);
	QCOMPARE(rotatePoint<90>(p), Point2d(0, 1));
	QCOMPARE(rotatePoint<180>(p), Point2d(-1, 0));
	QCOMPARE(rotatePoint<270>(p), Point2d(0, -1));
	double tst = 10 * M_PI / 180.0;
	double cosa = cos(tst);
	double sina = sin(tst);
	Point2d rotP = rotatePoint<10>(p);
	Point2d testP(cosa, sina);
	QCOMPARE(rotP.x,testP.x);
	QCOMPARE(rotP.y,testP.y);
	p.y = 1;
	p /= norm(p);
	QCOMPARE(rotatePoint<45>(p), Point2d(0, 1));
	QCOMPARE(rotatePoint<135>(p), Point2d(-1, 0));
	QCOMPARE(rotatePoint<225>(p), Point2d(0, -1));
	QCOMPARE(rotatePoint<315>(p), Point2d(1, 0));
}

QString LineHelpersTest::getErrStr(Point2d expected, Point2d result, float expectedWeight, float weight)
{
	return QString("Expected (%1, %2) w %3, got (%4, %5) w %6, dist %7").arg(expected.x)
			.arg(expected.y)
			.arg(expectedWeight)
			.arg(result.x)
			.arg(result.y)
			.arg(weight)
			.arg(norm(expected - result));
}

QTEST_APPLESS_MAIN(LineHelpersTest)
