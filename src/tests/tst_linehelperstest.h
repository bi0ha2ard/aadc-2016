#ifndef SR_LINEHELPERSTEST_H_
#define SR_LINEHELPERSTEST_H_
#include <QObject>

#include "../misc/LineHelpers.h"

class LineHelpersTest : public QObject
{
	Q_OBJECT

public:
	LineHelpersTest();

private Q_SLOTS:
	void initTestCase();
	void testCorrespPointLeftToLeft();
	void testCorrespPointLeftToRight();
	void testCorrespPointMiddleToLeft();
	void testCorrespPointRightToLeft();
	void testCorrespPointRightToRight();
	void testPointLeftMiddleLeft();
	void testCorrespOffsetLine();
	void testEmptyLine();
	void testSingle();
	void testFarAhead();
	void testFarBehind();
	void testFirstShortest();
	void testLastShortest();
	void testExtrapolate();
	void testOrtho();
	void testAngleCheck();
	void testPointInfrontof();
	void testPointDistToLine();
	void testFastRot();

private:
	QString getErrStr(Point2d expected, Point2d result, float expectedWeight, float weight);
	Line2d straightLine;
};

#endif
