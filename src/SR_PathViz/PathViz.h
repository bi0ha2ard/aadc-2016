#ifndef _PATH_VIZ_H_
#define _PATH_VIZ_H_

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <adtf_graphics.h>
#include <adtf_utils.h>

#include <odometry_data.h>
#include <QtGui>

#define OID_SR_PATHVIZ "sr.display.patviz"

class PathWidget : public QWidget
{
	Q_OBJECT
public:
	PathWidget(double startX, double startY, float scale, int imgSize);
	~PathWidget();
	void requestDraw(OdometryPose p);
	void requestGoalPose(ControllerCommand c);

private slots:
	void drawPose(double x, double y, double theta, double speed);
	void setGoalPose(double x, double y, double theta, double speed, int type);
	void hideGoalPose();

signals:
	void drawRequested(double x, double y, double theta, double speed);
	void setGoalPoseRequested(double x, double y, double theta, double speed, int type);

private:
	void drawGrid();

	QPixmap *image;
	QPixmap *backImage;
	QLabel *label;
	QLabel *title;
	QLabel *xPos;
	QLabel *yPos;
	QLabel *theta;
	QLabel *speed;
	QPushButton *hideGoalButton;
	QWidget *widget;
	int32_t offset;
	float scale;
	int imgSize;
	double startX;
	double startY;

	OdometryPose lastPose;
	ControllerCommand goalPose;
	ControllerCommand lastGoalPose;
	bool goalPoseChanged;
};



class PathViz : public adtf_graphics::cBaseQtFilter
{
	ADTF_FILTER(OID_SR_PATHVIZ, "SpaceRacer PathViz", adtf::OBJCAT_GraphicsDisplay)
public:
	PathViz(const tChar *info);
	virtual ~PathViz();

protected:
	tResult Init(tInitStage eStage, IException **__exception_ptr);
	tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);
	tHandle CreateView();
	tResult ReleaseView();

private:
	adtf::cInputPin inPin;
	adtf::cInputPin goalPosePin;
	PathWidget *widget;
};

#endif
