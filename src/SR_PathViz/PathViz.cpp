#include "PathViz.h"

#include <QtCore>


using namespace adtf;
using namespace adtf_graphics;

#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include "../misc/PinWrapper.h"

ADTF_FILTER_PLUGIN("SpaceRacer PathViz", OID_SR_PATHVIZ, PathViz)

#define PROP_SIZE "Map Size (px)"
#define PROP_SCALE "Pixels per meter"
#define PROP_STARTX "Starting x position"
#define PROP_STARTY "Starting y position"

PathViz::PathViz(const tChar *info) : cBaseQtFilter(info)
{
	SetPropertyInt(PROP_SIZE, 512);
	SetPropertyInt(PROP_SIZE NSSUBPROP_MIN, 128);
	SetPropertyInt(PROP_SCALE, 32);
	SetPropertyInt(PROP_SCALE NSSUBPROP_MIN, 1);
	SetPropertyFloat(PROP_STARTX, 0);
	SetPropertyFloat(PROP_STARTY, 0);
	widget = NULL;
}

PathViz::~PathViz()
{

}

tResult PathViz::Init(cFilter::tInitStage eStage, IException **__exception_ptr)
{
	RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst) {
		REGISTER_MEDIA_PIN(inPin, MEDIA_TYPE_ODOMETRY, MEDIA_SUBTYPE_ODOMETRY_POSE, "Pose");
		REGISTER_MEDIA_PIN(goalPosePin, MEDIA_TYPE_CONTROLLERCOMMAND, MEDIA_SUBTYPE_CONTROLLER_COMMAND, "GoalPose");
	} else if (eStage == StageNormal) {

	}
	RETURN_NOERROR;
}

tResult PathViz::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample)
{
	// Check pointers
	RETURN_IF_POINTER_NULL(pSource);
	RETURN_IF_POINTER_NULL(pMediaSample);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		if (pSource == &inPin) {
			OdometryPose p;
			READ_MEDIA_PIN(pMediaSample, OdometryPose, p);
			if (widget) {
				widget->requestDraw(p);
			}
		} else if (pSource == &goalPosePin) {
			ControllerCommand c;
			READ_MEDIA_PIN(pMediaSample, ControllerCommand, c);
			if (widget) {
				widget->requestGoalPose(c);
			}
		}
	}
	RETURN_NOERROR;
}

tHandle PathViz::CreateView()
{
	widget = new PathWidget(GetPropertyFloat(PROP_STARTX, 0), GetPropertyFloat(PROP_STARTY, 0), GetPropertyFloat(PROP_SCALE, 32), GetPropertyInt(PROP_SIZE, 512));
	return (tHandle)widget;
}

tResult PathViz::ReleaseView()
{
	RETURN_NOERROR;
}

void PathWidget::drawPose(double x, double y, double theta, double speed)
{
	xPos->setText(QString("x: %1 m").arg(x, 2, 'g', 3, QLatin1Char('0')));
	yPos->setText(QString("y: %1 m").arg(y, 2, 'g', 3, QLatin1Char('0')));
	this->theta->setText(QString::fromUtf8("\u03B8: %1\u00B0").arg(180.0 * theta / M_PI, 2, 'g', 3, QLatin1Char('0')));
	this->speed->setText(QString("v: %1 m/s").arg(speed, 2, 'g', 3, QLatin1Char('0')));
	if (!image) {
		return;
	}

	{
		QPainter painter(backImage);
		QTransform t;
		t.setMatrix(1, 0, 0, 0, 1, 0, offset + startX * scale, offset - startY * scale, 1);
		painter.setTransform(t);
		painter.setPen(Qt::red);
		painter.drawLine(QPointF(lastPose.x * scale, -lastPose.y * scale), QPointF(x * scale, -y * scale));
		if (goalPoseChanged) {
			if (lastGoalPose.type != DesiredPose) {
				painter.setPen(Qt::cyan);
				painter.drawEllipse(lastGoalPose.pose.x * scale - 2, -lastGoalPose.pose.y * scale - 2, 4, 4);
				double len = .5 + lastGoalPose.pose.speed;
				painter.drawLine(lastGoalPose.pose.x * scale, - lastGoalPose.pose.y * scale, lastGoalPose.pose.x * scale + cos(lastGoalPose.pose.theta) * scale * len, -lastGoalPose.pose.y * scale - sin(lastGoalPose.pose.theta) * scale * len);
			} else {
				painter.setPen(Qt::darkGreen);
				painter.drawEllipse(lastGoalPose.pose.x * scale - 2, -lastGoalPose.pose.y * scale - 2, 4, 4);
			}
			goalPoseChanged = false;
		}
	}
	*image = backImage->copy();
	{
		QPainter painter(image);
		QTransform t;
		t.setMatrix(1, 0, 0, 0, 1, 0, offset + startX * scale, offset - startY * scale, 1);
		painter.setTransform(t);
		painter.setPen(Qt::magenta);
		painter.drawEllipse(x * scale - 2, -y * scale - 2, 4, 4);
		painter.drawLine(x * scale, - y * scale, x * scale + cos(theta) * scale * (.5 + speed), -y * scale - sin(theta) * scale * (.5 + speed));
		if (hideGoalButton->isVisible()) {
			QPen pen(goalPose.type == DesiredPose ? Qt::darkGreen : Qt::cyan);
			pen.setWidth(3);
			painter.setPen(pen);
			double len = .5 + goalPose.pose.speed;
			painter.drawEllipse(goalPose.pose.x * scale - 4, -goalPose.pose.y * scale - 4, 8, 8);
			painter.drawLine(goalPose.pose.x * scale, - goalPose.pose.y * scale, goalPose.pose.x * scale + cos(goalPose.pose.theta) * scale * len, -goalPose.pose.y * scale - sin(goalPose.pose.theta) * scale * len);
		}
	}
	label->setPixmap(*image);
	lastPose.x = x;
	lastPose.y = y;
	lastPose.theta = theta;
	lastPose.speed = speed;
}

void PathWidget::setGoalPose(double x, double y, double theta, double speed, int type)
{
	goalPoseChanged = true;
	lastGoalPose = goalPose;
	goalPose.pose.x = x;
	goalPose.pose.y = y;
	goalPose.pose.theta = theta;
	goalPose.pose.speed = speed;
	goalPose.type = static_cast<ControllerCommandType>(type);
	hideGoalButton->setVisible(true);
}

void PathWidget::hideGoalPose()
{
	hideGoalButton->setHidden(true);
}

void PathWidget::drawGrid()
{
	if (!backImage) {
		return;
	}
	backImage->fill(Qt::white);
	QPainter p(backImage);
	// Draw grid
	p.setBrush(Qt::gray);
	p.setPen(Qt::DotLine);
	for (int i = scale; i < backImage->height(); i += scale) {
		p.drawLine(0, i, backImage->width(), i);
	}
	for (int i = scale; i < backImage->width(); i += scale) {
		p.drawLine(i, 0, i, backImage->height());
	}
	// Draw coord system
	p.setBrush(Qt::black);
	p.setPen(Qt::SolidLine);
	p.drawLine(0, offset, backImage->width(), offset);
	p.drawLine(offset, 0, offset, backImage->height());
	p.drawText(offset, 20, "Y");
	p.drawText(backImage->width() - 20, offset, "X");
}

PathWidget::PathWidget(double startX, double startY, float scale, int imgSize)
{
	this->startX = startX;
	this->startY = startY;
	this->scale = scale;
	this->imgSize = imgSize;
	offset = static_cast<int>((imgSize / scale / 2) * scale);

	image = new QPixmap(imgSize,imgSize);
	backImage = new QPixmap(imgSize,imgSize);
	drawGrid();

	lastPose.x = 0;
	lastPose.y = 0;
	lastPose.theta = 0;
	lastPose.speed = 0;
	lastGoalPose.pose = lastPose;
	lastGoalPose.type = DesiredPose;
	goalPose = lastGoalPose;
	goalPoseChanged = false;

	label = new QLabel();
	label->setPixmap(*backImage);
	title = new QLabel(QString("Scale: %1 p/m (Gridspacing = 1m)").arg(scale));
	hideGoalButton = new QPushButton("x");
	hideGoalButton->setMaximumWidth(50);
	hideGoalButton->setToolTip("Hide goal pose");
	QBoxLayout *layout = new QBoxLayout(QBoxLayout::TopToBottom);
	QBoxLayout *titleLayout = new QBoxLayout(QBoxLayout::LeftToRight);
	titleLayout->addWidget(title);
	titleLayout->addWidget(hideGoalButton);
	layout->addLayout(titleLayout);
	QBoxLayout *dispLayout = new QBoxLayout(QBoxLayout::LeftToRight);
	xPos = new QLabel();
	yPos = new QLabel();
	theta = new QLabel();
	speed = new QLabel();
	dispLayout->addWidget(xPos);
	dispLayout->addWidget(yPos);
	dispLayout->addWidget(theta);
	dispLayout->addWidget(speed);
	layout->addLayout(dispLayout);
	layout->addWidget(label);
	this->setLayout(layout);
	connect(this, SIGNAL(drawRequested(double ,double, double, double)), SLOT(drawPose(double, double, double, double)), Qt::QueuedConnection);
	connect(this, SIGNAL(setGoalPoseRequested(double ,double, double, double, int)), SLOT(setGoalPose(double, double, double, double, int)), Qt::QueuedConnection);
	hideGoalButton->setVisible(false);
	connect(hideGoalButton, SIGNAL(clicked()), SLOT(hideGoalPose()));
}

PathWidget::~PathWidget()
{
	delete image;
	delete backImage;
}

void PathWidget::requestDraw(OdometryPose p)
{
	emit drawRequested(p.x, p.y,p.theta * M_PI / 180.0, p.speed);
}

void PathWidget::requestGoalPose(ControllerCommand c)
{
	emit setGoalPoseRequested(c.pose.x, c.pose.y, c.pose.theta * M_PI / 180.0, c.pose.speed, static_cast<int>(c.type));
}
