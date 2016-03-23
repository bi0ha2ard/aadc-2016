#include "ProcessingThread.h"
#include "SR_ImageProcessing.h"
#include <mutexlocker.h>
#include "../misc/LineHelpers.h"
#include "../misc/FastRot.h"
#include "../misc/OdometryHelpers.h"
#include "../misc/CrossingHelpers.h"

using namespace adtf;
using namespace cv;

#undef LOG_INFO
#define LOG_INFO(str) SR_ImageProcessing::logger.GLogTrap(logName + (str));
#define LOG_LANE(str) if (prefs.debugLaneFinder) {LOG_INFO(str)}

ProcessingThread::ProcessingThread(const char *name, SR_ImageProcessing *imageProcessing) :
	name(name),
	centerLineMaxAge(20)
{
	logName = this->name + ": ";
	newData = false;
	shouldrun = false;
	event.Create();
	prevStartTime.tv_nsec = 0;
	prevStartTime.tv_sec = 0;
	startTime = prevStartTime;
	endTime = prevStartTime;
	this->imageProcessing = imageProcessing;

	prefs.outputType = SR_ImageProcessing::Threshold;

	prefs.debugPrintoutEnabled = false;
	prefs.debugCrossings = false;
	prefs.debugTimings = false;

	prefs.useAdaptiveThreshold = false;
	prefs.autoThreshold = true;
	prefs.minContourArea = 100;

	prefs.dilationKernelW = 5;
	prefs.dilationKernelH = 5;

	prefs.laneCenterLength = .3f;
	prefs.laneCenterDev = .08f;
	prefs.laneCenterAsp = 3.5;

	prefs.enableCrossingDetection = true;
	prefs.enableCrossingExitDetection = true;

	prefs.pParkingWidth = .7;
	prefs.pParkingWidthDev = .1;

	prefs.meterPerPixel = 1;

	validCrossingRegion.push_back(Point2i(120, 300));
	validCrossingRegion.push_back(Point2i(120, 480));
	validCrossingRegion.push_back(Point2i(380, 480));
	validCrossingRegion.push_back(Point2i(380, 300));

	imgBuf = NULL;
	imgBuf = (char*)malloc(640*480*3);
	imageProcessing->logger.log() << "Worker created " << name << endl;

	shouldResetRight = false;
}

ProcessingThread::~ProcessingThread()
{
	if (imgBuf) {
		free(imgBuf);
	}
	imageProcessing->logger.log() << "Worker destroyed " << name << endl;
}

void ProcessingThread::requestProcessing(adtf::IMediaSample *s, tTimeStamp timeStamp, OdometryPose p)
{
	//LOG_INFO(cString::Format("%s: request", name));
	dataMutex.Enter();
	currTimeStamp = timeStamp;
	const tVoid* l_pSrcBuffer;
	s->Lock(&l_pSrcBuffer);
	memcpy(imgBuf, l_pSrcBuffer, 640*480*3);
	s->Unlock(l_pSrcBuffer);
	IplImage* oImg = cvCreateImageHeader(cvSize(640, 480), IPL_DEPTH_8U, 3);
	oImg->imageData = imgBuf;
	data = Mat(cvarrToMat(oImg));
	cvReleaseImage(&oImg);
	newData = true;
	currentPose = p;
	dataMutex.Leave();
	event.Set();
}

tResult ProcessingThread::createThread()
{
	return thread.Create(cKernelThread::TF_Suspended, this, NULL, 0, name);
}

tResult ProcessingThread::shutdownThread()
{
	shouldrun = false;
	RETURN_IF_FAILED(thread.Terminate(true));
	return thread.Release();
}

tResult ProcessingThread::stopThread()
{
	shouldrun = false;
	return thread.Suspend(true);
}

tResult ProcessingThread::runThread()
{
	shouldrun = true;
	return thread.Run();
}

int searchPointDist = 0;

void ProcessingThread::testExtrapolation()
{
	searchPointDist = 10;
	vector<Point2i> lane;
	Point2d searchPoint(320, 480);
	Point2d searchNormal(1, 0);
	Point2d extPoint = getExtrapolatedPoint(lane, searchPoint, searchNormal);
	imageProcessing->logger.log() << "Lane size " << lane.size() << " Search Point: " << searchPoint << " " << " Extrapolated point " << extPoint << endl;
	lane.push_back(searchPoint);
	extPoint = getExtrapolatedPoint(lane, searchPoint, searchNormal);
	imageProcessing->logger.log() << "Lane size " << lane.size() << " Search Point: " << searchPoint << " " << " Extrapolated point " << extPoint << endl;
	lane.push_back(Point2i(320, 470));
	extPoint = getExtrapolatedPoint(lane, searchPoint, searchNormal);
	imageProcessing->logger.log() << "Lane size " << lane.size() << " Search Point: " << searchPoint << " " << " Extrapolated point " << extPoint << endl;
	lane.push_back(Point2i(320, 460));
	extPoint = getExtrapolatedPoint(lane, searchPoint, searchNormal);
	imageProcessing->logger.log() << "Lane size " << lane.size() << " Search Point: " << searchPoint << " " << " Extrapolated point " << extPoint << endl;
}

void ProcessingThread::resetLaneTracking()
{
	resetMutex.lock();
	shouldResetRight = true;
	shouldResetLeft = true;
	resetMutex.unlock();
	LOG_LANE("Reset tracking");
}

void ProcessingThread::resetLaneTracking(ProcessingThread::LaneSearchType type)
{
	resetMutex.lock();
	if (type == RightLaneSearch) {
		shouldResetRight = true;
	} else {
		shouldResetLeft = true;
	}
	resetMutex.unlock();
	LOG_LANE("Reset tracking");
}
#define TIMEDIFF(startTime, endTime) static_cast<uint64_t>((((endTime).tv_sec * 1000000000 + (endTime).tv_nsec) - ((startTime).tv_sec * 1000000000 + (startTime).tv_nsec)) / 1000000)

tResult ProcessingThread::ThreadFunc(adtf::cKernelThread *pThread, tVoid *pvUserData, tSize szUserData)
{
	while (shouldrun) {
		event.Wait(200);
		if (prefs.debugTimings) {
			clock_gettime(CLOCK_REALTIME, &startTime);
			LOG_INFO(cString::Format("Time since last frame: %lu ms", (TIMEDIFF(prevStartTime, startTime))));
			prevStartTime = startTime;
		}
		prefs = imageProcessing->getPrefs();
		updateTransform();
		process();
		if (prefs.debugTimings) {
			clock_gettime(CLOCK_REALTIME, &endTime);
			LOG_INFO(cString::Format("took %lu ms to process frame", TIMEDIFF(startTime, endTime)));
		}
		event.Reset();
	}
	RETURN_NOERROR;
}

bool ProcessingThread::featureLowerThan(const Feature *a, const Feature *b)
{
	return a->rect.center.y > b->rect.center.y;
}

tResult ProcessingThread::process()
{
	if (data.total() == 0) {
		LOG_INFO("Data is totes empty");
		RETURN_NOERROR;
	}
	if (!newData) {
		RETURN_NOERROR;
	}
	newData = false;

	setConstants();

	if (prefs.outputType == SR_ImageProcessing::ReversedOrder) {
		dataMutex.Enter();
		imageProcessing->applyHomography(&data, &m_TopDown);
		cvtColor(data, m_matGreyUncut, CV_RGB2GRAY);
		dataMutex.Leave();
		threshold(m_matGreyUncut, m_matGreyThreshUncut, SR_ImageProcessing::thresholdValue, 500,THRESH_BINARY);// Generate Binary Image
		Mat kernel = cv::Mat::ones(prefs.dilationKernelH, prefs.dilationKernelW, CV_8UC1);
		// Names don't correspond to images anymore but i need dilated to be the top down image
		dilate(m_matGreyThreshUncut, m_ImageUncut, kernel);
		imageProcessing->applyHomography(&m_ImageUncut, &dilated);
		dilated.copyTo(m_matGreyThreshUncut);
		processCanny();
		m_Output = m_Drawing;
	} else {
		dataMutex.Enter();
		imageProcessing->applyHomography(&data, &m_TopDown);
		dataMutex.Leave();
		if (prefs.outputType == SR_ImageProcessing::Greyscale) {
			cvtColor(m_TopDown, m_Output, CV_RGB2GRAY);
			// Debug
			m_TopDown = data;
		} else if (prefs.outputType == SR_ImageProcessing::Threshold || prefs.outputType == SR_ImageProcessing::Canny) {
			GaussianBlur(m_TopDown, m_ImageUncut, Size(15,15), 2, 0, BORDER_DEFAULT); // Filter
			cvtColor(m_ImageUncut, m_matGreyUncut ,CV_RGB2GRAY);// Grey Image
			if (prefs.useAdaptiveThreshold) {
				cv::adaptiveThreshold(m_matGreyUncut, m_matGreyThreshUncut, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 7, -5);
			} else {
				threshold(m_matGreyUncut, m_matGreyThreshUncut, SR_ImageProcessing::thresholdValue, 500,THRESH_BINARY);// Generate Binary Image
			}
			if (prefs.outputType == SR_ImageProcessing::Canny) {
				// Grow regions
				Mat kernel = cv::Mat::ones(prefs.dilationKernelH, prefs.dilationKernelW, CV_8UC1);
				dilate(m_matGreyThreshUncut, dilated, kernel);
				dilated.copyTo(m_matGreyThreshUncut);

				processCanny();
				m_Output = m_Drawing;
			} else {
				m_Output = m_matGreyThreshUncut;
			}
		}
	}

	// DEBUG
	if (prefs.debugDrivingPath && imageProcessing->shouldOutputTopDown()) {
		drawDrivingPath();
	}

	if (imageProcessing->shouldOutputFiltered()) {
		imageProcessing->transmitFilteredImage(m_Output);
	}
	if (imageProcessing->shouldOutputTopDown()) {
		imageProcessing->transmitTopDownImage(m_TopDown);
	}
	if (imageProcessing->shouldOutputThreshold()) {
		imageProcessing->transmitThresholdImage(m_matGreyThreshUncut);
	}
	lastPose = currentPose;
	RETURN_NOERROR;
}

tResult ProcessingThread::processCanny()
{
	bool shouldOutput = imageProcessing->shouldOutputFiltered();
	if (prefs.debugCrossings) {
		vector<vector<Point2i> >regionRegion;
		regionRegion.push_back(validCrossingRegion);
		drawContours(m_TopDown, regionRegion, 0, CV_RGB(128, 0, 0), 1, CV_AA);
	}

	// After this point, old left, right and center lanes are present
	updateOldLanes();

	// Find contours
	vector<vector<Point> > contours;
	if (shouldOutput) {
		m_Drawing = Mat::zeros(data.size(),CV_8UC1 );
	}
	findContours(dilated, contours, cv::noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );

	int smallContours = 0;
	int bigContours = 0;
	int giantContours = 0;

	// TODO combine?
	prevFeatures = currFeatures;
	resetLaneData();

	// Filter contours
	for(size_t i = 0; i < contours.size(); i++){
		Feature f = generateFeature(contours, i);
		if (f.cArea < prefs.minContourArea) {
			// skip small stuff
			++smallContours;
			continue;
		} else if (f.cArea > 10000) {
			++giantContours;
		}
		++bigContours;

		currFeatures.push_back(f);

		if (shouldOutput) {
			drawContours(m_Drawing, contours, i, Scalar(128), 1, 8, vector<Vec4i>(), 0, Point());
		}
	}

	// Extract center Features
	for (size_t i = 0; i < currFeatures.size(); ++i) {
		checkCenterFeature(currFeatures[i]);
	}

	// Generate center lane
	Line2i temp = extractCenterLane();
	if (!temp.empty()) {
		center = extractCenterLane();
	} else {
		center = oldCenter;
	}
	//fitCenterLine();

	// Generate left, right lane
	findLeftRightLanes();

	// Send everything to PathPlanner
	generateAndSendLaneData();

	// find crossings
	if (prefs.enableCrossingDetection) {
		for (size_t i = 0; i < currFeatures.size(); ++i) {
			findCorners(currFeatures[i], contours);
		}
		extractCrossings();
	}

	if (prefs.debugPrintoutEnabled) {
		for (size_t i = 0; i < currFeatures.size(); ++i) {
			LOG_INFO(cString::Format("%d: Type: %d long %g m (%g pix) short %g m (%g pix) aspect %g angle %g, w %g, h %g, radiusEstimate %g m (%g pix), area %g", currFeatures[i].contour_idx, currFeatures[i].type, currFeatures[i].longSide * prefs.meterPerPixel, currFeatures[i].longSide, currFeatures[i].shortSide * prefs.meterPerPixel, currFeatures[i].shortSide, currFeatures[i].aspectRatio, currFeatures[i].rect.angle, currFeatures[i].rect.size.width, currFeatures[i].rect.size.height, currFeatures[i].radiusEstimate * prefs.meterPerPixel, currFeatures[i].radiusEstimate, currFeatures[i].cArea));
		}
	}

	if (imageProcessing->shouldOutputTopDown()) {
		debugDrawFeatures();
	}


	findParkingSlot();

	if (prefs.autoThreshold) {
		// autoexposure
		if (smallContours > 20 || giantContours > 1) {
			++SR_ImageProcessing::thresholdValue;
			if (prefs.debugPrintoutEnabled) {
				LOG_INFO(cString::Format("Thresh up: %d", SR_ImageProcessing::thresholdValue));
			}
		}
		if (bigContours < 1) {
			--SR_ImageProcessing::thresholdValue;
			if (prefs.debugPrintoutEnabled) {
				LOG_INFO(cString::Format("Thresh down: %d", SR_ImageProcessing::thresholdValue));
			}
		}
		if (contours.size() - smallContours < 2 && giantContours == 0) {
			// TODO handle curve
			--SR_ImageProcessing::thresholdValue;
			if (prefs.debugPrintoutEnabled) {
				LOG_INFO(cString::Format("Thresh down too few contours: %d", SR_ImageProcessing::thresholdValue));
			}
		}
	}
	RETURN_NOERROR;
}

ProcessingThread::Feature ProcessingThread::generateFeature(const vector<vector<Point> > &contours, size_t i)
{
	Feature f;
	f.type = Garbage;
	f.contour_idx = i;
	f.cArea = contourArea(contours[i]);
	f.rect = minAreaRect(contours[i]);
	if (f.rect.size.width > f.rect.size.height) {
		f.rect.angle += 90;
		float tmp = f.rect.size.height;
		f.rect.size.height = f.rect.size.width;
		f.rect.size.width = tmp;
	}
	f.aspectRatio = fmax(f.rect.size.width / f.rect.size.height, f.rect.size.height / f.rect.size.width);
	f.longSide = fmax(f.rect.size.width, f.rect.size.height);
	f.shortSide = fmin(f.rect.size.width, f.rect.size.height);
	return f;
}

void ProcessingThread::resetLaneData()
{
	prevFeatures.clear();
	currFeatures.clear();
	centerLane.clear();
	crossingCandidates.clear();
}

bool ProcessingThread::isCenterLine(const ProcessingThread::Feature &f) const
{
	if (fabs(f.longSide - laneCenterLPix) < laneCenterLDev && f.aspectRatio > prefs.laneCenterAsp) {
		// it's a middle lane stripe
		return true;
	}
	double distTraveled = 1.5 * norm(getOdometryPos(lastPose) - getOdometryPos(currentPose)) / prefs.meterPerPixel;
	if (f.type == Garbage) {
		for (size_t k = 0; k < prevFeatures.size(); ++k) {
			if (prevFeatures[k].type == Center && norm(prevFeatures[k].rect.center - f.rect.center) < distTraveled && fabs(prevFeatures[k].cArea - f.cArea) < 100) {
				return true;
			}
		}
	}
	return false;
}

bool ProcessingThread::findCurve(const vector<Point> &contour, ProcessingThread::Feature &f)
{
	double angRad = M_PI * f.rect.angle / 180.0;
	Point2f normal;
	normal.x = cos(angRad);
	normal.y = sin(angRad);
	if (angRad > 0) {
		normal *= -1;
	}
	// outside is negative
	double outerDist;
	double innerDist;
	getDistanceFromSide(contour, f, &innerDist, &outerDist, &normal);
	if (innerDist < 2) {
		// Not a curve?
	}
	double h = innerDist + (outerDist - innerDist) / 2.0;
	f.radiusEstimate = h / 2.0 + f.longSide * f.longSide / (8.0 * h);

	f.vecToCenter = normal * (f.radiusEstimate + (.5 * f.shortSide - h));
	f.curveCenter = f.rect.center - f.vecToCenter;
	if (imageProcessing->shouldOutputTopDown()) {
		cv::circle(m_TopDown, f.curveCenter, f.radiusEstimate, CV_RGB(255, 0, 0), 1, CV_AA);
		cv::line(m_TopDown, f.curveCenter, f.curveCenter + normal * f.radiusEstimate, CV_RGB(255, 0, 0), 1, CV_AA);
	}

	if (fabs(f.radiusEstimate - 2.0 / prefs.meterPerPixel) < 20 || fabs(f.radiusEstimate - 3.0 / prefs.meterPerPixel) < 20 || fabs(f.radiusEstimate - 1.0 / prefs.meterPerPixel)) {
		// Angle is negative for left curve
		if (f.rect.center.x > 320 + 2 * f.rect.angle) {
			// right lane?
			f.type = RightLaneCurve;
		} else if (f.rect.center.x < 240 + 2 * f.rect.angle){
			// left lane?
			f.type = LeftLaneCurve;
		} else {
			// wut?
		}
	}
	return f.type != Garbage;
}

bool ProcessingThread::findCorners(ProcessingThread::Feature &f, vector<vector<Point> > &contours)
{
#if 0
	for (size_t k; k < contours[i].size(); ++k) {
		cv::circle(m_Drawing, contours[i][k], 2, Scalar(255 * k / contours[i].size()), 1, CV_AA);
	}
#endif
	size_t i = f.contour_idx;
	if (f.type != Garbage) {
		return false;
	}
	if (f.cArea > 8000) {
		return false;
	}
	bool corners = false;
	vector<int> idx;
	cv::convexHull(contours[i], idx);
	vector<Vec4i> defects;
	if (idx.size() < 3) {
		return false;
	}

#if 0
	// convexity defects
	cv::convexityDefects(contours[i], idx, defects);
	for (size_t k = 0; k < defects.size(); ++k) {
		if (defects[k][3] < 2560) {
			// defect less than 10 pix
			continue;
		}
		Point a = contours[i][defects[k][0]];
		Point b = contours[i][defects[k][1]];
		cv::line(m_Drawing, a, b, Scalar(200), 2, CV_AA);
		cv::line(m_Drawing, a + (b - a) * .5, contours[i][defects[k][2]], Scalar(200), 2, CV_AA);
	}
#endif

	std::vector<Point> l1;
	std::vector<Point> l2;
	Vec4f l1res;
	Vec4f l2res;
	for (size_t k = 0; k < idx.size(); ++k) {
		if (!insideImageRegion(contours[i][idx[k]])) {
			continue;
		}
		// TODO abort early
		Point anchor = contours[i][idx[k]];
		int aa = 1;
		int bb = 1;
		l1.clear();
		l2.clear();
		l1.push_back(contours[i][idx[k]]);
		l2.push_back(contours[i][idx[k]]);
		// Contours are sorted, so this works
		do {
			l1.push_back(contours[i][(idx[k] - aa + contours[i].size()) % contours[i].size()]);
			++aa;
		} while (aa < 5);
		do {
			l2.push_back(contours[i][(idx[k] + bb) % contours[i].size()]);
			++bb;
		} while (bb < 5);
		cv::fitLine(l1, l1res, CV_DIST_L2, 0, .01, .01);
		cv::fitLine(l2, l2res, CV_DIST_L2, 0, .01, .01);
		double angle1 = atan2(l1res[1], l1res[0]);
		double angle2 = atan2(l2res[1], l2res[0]);
		double angle =  angle1 - angle2;
		while (angle < 0) {
			angle += 2 * M_PI;
		}
		while (angle > 2 * M_PI) {
			angle -= 2 * M_PI;
		}
		if (fabs(angle - M_PI_2) < .2 || fabs(angle - (3.0 * M_PI_2)) < .2) {
			// 90 or 270 deg
			// Find length of straight segments at corner
			while (static_cast<size_t>(aa) < contours[i].size() / 2) {
				Point p = contours[i][(idx[k] - aa + contours[i].size()) % contours[i].size()];
				if (distSquare(p, l1.back()) < 64) {
					++aa;
					continue;
				}
				Point vec = p - l1.back();
				Point vecLong = p - anchor;
				if (isParallel(atan2(vecLong.y, vecLong.x), atan2(vec.y, vec.x), .3)) {
					l1.push_back(p);
				} else {
					break;
				}
				++aa;
			}
			while (static_cast<size_t>(bb) < contours[i].size() / 2) {
				Point p = contours[i][(idx[k] + bb) % contours[i].size()];
				if (distSquare(p, l2.back()) < 64) {
					++bb;
					continue;
				}
				Point vec = p - l2.back();
				Point vecLong = p - anchor;
				if (isParallel(atan2(vecLong.y, vecLong.x), atan2(vec.y, vec.x), .3)) {
					l2.push_back(p);
				} else {
					break;
				}
				++bb;
			}

			if (imageProcessing->shouldOutputTopDown()) {
				cv::circle(m_TopDown, contours[i][idx[k]], 3, CV_RGB(255, 0, 0), 1, CV_AA);
				cv::line(m_TopDown, contours[i][idx[k]], l1.back(), CV_RGB(0, 255, 0), 1, CV_AA);
				cv::line(m_TopDown, contours[i][idx[k]], l2.back(), CV_RGB(0, 255, 40), 1, CV_AA);
				cv::circle(m_Drawing, contours[i][idx[k]], 3, Scalar(255), 1, CV_AA);
			}

			corners = true;
			Point2d side1 = l1.back() - anchor;
			Point2d side2 = l2.back() - anchor;
			double sl1 = norm(side1);
			double sl2 = norm(side2);
			if (max(sl1, sl2) > crossingCornerL2Pix && min(sl1, sl2) > crossingCornerL1Pix) {
				// it's a crossing
				Point2d normal = (side1 / sl1) + (side2 / sl2);
				CrossingCandidate cc = findCrossingExitsFromCorner(anchor, normal, l1.back(), l2.back());
				double alpha1 = atan2(side1.y, side1.x);
				double alpha2 = atan2(side2.y, side2.x);
				double halfAngle = (fmax(alpha1, alpha2) - fmin(alpha2, alpha1)) / 2.0;
				if (halfAngle > M_PI_2) {
					// correct for discontinuity at 180 deg
					halfAngle = M_PI - halfAngle;
				}
				if (halfAngle < M_PI_2 / 3.0) {
					// skip stuff that isn't 90 deg after growing the lines
					continue;
				}
				// center
				if (!insideImageRegion(cc.crossingCenter)) {
					if (prefs.debugCrossings) {
						LOG_INFO(cString::Format("Discarding crossing for contour %d, outside image area", f.contour_idx));
					}
					continue;
				}
				if (cc.exits != 0xf && prefs.enableCrossingExitDetection) {
					uint8_t tmp = findCrossingExitsFromImage(cc);
					uint8_t old = cc.exits;
					cc.exits |= tmp;
					cc.exits |= ~cc.blockedExits;
					cc.exits &= 0xf; // mask out everything but last 4 bits
					if (prefs.debugCrossings) {
						imageProcessing->logger.log() << "Corner exits " << hex << (int) old << " image exits " << (int) tmp << " blocked exits " << (int) cc.blockedExits << " final " << (int)cc.exits << dec << endl;
					}
				} else if (!prefs.enableCrossingExitDetection) {
					// Enable all crossins
					cc.exits |= 0xf;
				}
				f.curveCenter = cc.crossingCenter;
				f.type = Crossing;
				f.vecToCenter = cc.crossingNormal;
				bool onStreet = pointOnStreet(cc.crossingCenter);
				bool insideRegion = insideCrossingRegion(cc.crossingCenter);
				if (onStreet || insideRegion) {
					if (imageProcessing->shouldOutputTopDown()) {
						cv::circle(m_TopDown, f.curveCenter, 10, CV_RGB(255, 0, 0), 2, CV_AA);
						cv::line(m_TopDown, anchor, f.curveCenter, CV_RGB(255, 0, 0), 1, CV_AA);
						cv::arrowedLine(m_TopDown, cc.crossingCenter, cc.crossingCenter + 20 * cc.crossingNormal, CV_RGB(255, 0, 0), 2, CV_AA);
					}
					if (prefs.debugCrossings) {
						imageProcessing->logger.log() << "Crossing " << cc.crossingCenter << " on street " << onStreet << " in region " << insideRegion << endl;
					}
					crossingCandidates.push_back(cc);
				} else {
					if (imageProcessing->shouldOutputTopDown()) {
						cv::circle(m_TopDown, f.curveCenter, 10, CV_RGB(0, 0, 255), 2, CV_AA);
						cv::line(m_TopDown, anchor, f.curveCenter, CV_RGB(0, 0, 255), 1, CV_AA);
					}
					if (prefs.debugCrossings) {
						imageProcessing->logger.log() << "Rejecting contour at " << cc.crossingCenter << " because it is not on the street and outside the crossing region " << endl;
					}
				}
			}
		}
	}
	return corners;
}

ProcessingThread::CrossingCandidate ProcessingThread::findCrossingExitsFromCorner(const cv::Point2d &anchor, cv::Point2d normal, const cv::Point2d &l1back, const cv::Point2d &l2back)
{
	CrossingCandidate cc;
	cc.exits = 0;
	cc.blockedExits = 0;
	cc.exits |= EXIT_SOUTH;
	normal /= -norm(normal);
	normal *= crossingCenterDist; // Half the diagonal of the crossing, points to center
	circle(m_TopDown, l1back, 3, CV_RGB(0, 255, 255), 1);
	if (normal.x < 0 && normal.y < 0) {
		// Bottom right corner
		bool found = false;
		if (hasStopLine(right, center, left, anchor) || hasStopLine(oldRight, oldCenter, oldLeft, anchor)) {
			normal = makeStopLineCrossingNormal(anchor, l2back);
			cc.crossingNormal = normal;
			cc.stopLineOnTrack = true;
			if (prefs.debugCrossings) {
				LOG_INFO("Stop line in our direction");
			}
			found = true;
		}
		if (!found) {
			Point2d sNorm = l1back - anchor;
			bool outside = false;
			double dist = distToWhite(l2back, sNorm, l20cm, l60cm, outside);
			if (!outside && dist < l50cm) {
				normal = makeStopLineCrossingNormal(anchor, l2back);
				cc.crossingNormal = normal;
				cc.stopLineOnTrack = true;
				if (prefs.debugCrossings) {
					LOG_INFO("Stop line in our direction from img");
				}
				found = true;
			}
		}
		if (!found) {
			cc.exits |= EXIT_RIGHT;
			getRotationMatrix2D(Point2f(0, 0), M_PI_4, 1);
			cc.crossingNormal = rotatePoint<45>(normal);
			if (prefs.debugCrossings) {
				LOG_INFO("Crossing exit bottom right");
			}
		}
	} else if (normal.x > 0 && normal.y > 0) {
		// Top left corner
		cc.exits |= EXIT_STRAIGHT; // Always has exit straight
		bool found = false;
		if (hasStopLine(left, center, right, anchor) || hasStopLine(oldLeft, oldCenter, oldRight, anchor)) {
			normal = makeStopLineCrossingNormal(anchor, l2back);
			cc.crossingNormal = rotatePoint<180>(normal);
			if (prefs.debugCrossings) {
				LOG_INFO("Stop line in oncoming lane");
			}
			found = true;
		}
		if (!found) {
			Point2d sNorm = l1back - anchor;
			bool outside = false;
			double dist = distToWhite(l2back, sNorm, l20cm, l60cm, outside);
			if (!outside && dist < l50cm) {
				normal = makeStopLineCrossingNormal(anchor, l2back);
				cc.crossingNormal = rotatePoint<180>(normal);
				if (prefs.debugCrossings) {
					LOG_INFO("Stop line in our direction from img");
				}
				found = true;
			}
		}
		if (!found) {
			if (prefs.debugCrossings) {
				LOG_INFO("Crossing exit top left");
			}
			cc.exits |= EXIT_LEFT;
			cc.crossingNormal = rotatePoint<225>(normal);
		}
	} else if (normal.x < 0 && normal.y > 0) {
		// Top right
		cc.exits |= EXIT_RIGHT;
		Point2d sNorm = l1back - anchor;
		bool outside = false;
		double dist = distToWhite(l2back, sNorm, l20cm, l60cm, outside);
		if (!outside && dist < l50cm) {
			if (prefs.debugCrossings) {
				LOG_INFO("Stop line right from img");
			}
			normal = makeStopLineCrossingNormal(anchor, l2back);
			cc.crossingNormal = rotatePoint<90>(normal);
		} else {
			// No stop line
			if (prefs.debugCrossings) {
				LOG_INFO("Crossing exit top right");
			}
			cc.exits |= EXIT_NORTH;
			cc.crossingNormal = rotatePoint<135>(normal);
		}
	} else if (normal.x > 0 && normal.y < 0) {
		// Bottom left
		cc.exits |= EXIT_LEFT;
		Point2d sNorm = anchor - l1back;
		bool outside = false;
		double dist = distToWhite(l2back, sNorm, l20cm, l60cm, outside);
		if (!outside && dist < l50cm) {
			if (prefs.debugCrossings) {
				LOG_INFO("Stop line left");
			}
			normal = makeStopLineCrossingNormal(anchor, l2back);
			cc.crossingNormal = rotatePoint<270>(normal);
		} else {
			if (prefs.debugCrossings) {
				LOG_INFO("Crossing exit bottom left");
			}
			cc.crossingNormal = rotatePoint<315>(normal);
		}
	}

	cc.crossingCenter = Point2d(anchor.x, anchor.y) + normal;
	cc.crossingNormal /= norm(cc.crossingNormal);
	return cc;
}

uint8_t ProcessingThread::findCrossingExitsFromImage(CrossingCandidate &cc)
{
	uint8_t exits = 0;
	bool outsideRegion = false;
	double offs = l20cm;
	double minDist = crossingExitMinFreeDist;
	int start = l20cm; // skip over potential center lanes
	// right exit
	Point2d anchor = cc.crossingCenter + Point2d(0, offs);
	Point2d normal(1, 0);
	int dist = 0;
	if (!(HAS_EXIT_RIGHT(cc))) {
		dist = distToWhite(anchor, normal, start, crossingExitMaxDist, outsideRegion);
		if (dist > minDist) {
			exits |= EXIT_RIGHT;
			if (prefs.debugCrossings) {
				cv::line(m_TopDown, anchor + normal * start, anchor + normal * dist, CV_RGB(0, 0, 255), 2, CV_AA);
				cv::circle(m_TopDown, anchor + normal * minDist, 2, CV_RGB(0, 0, 255), 2, CV_AA);
				LOG_INFO("Right exit from image");
			}
		} else {
			cc.blockedExits |= EXIT_RIGHT;
		}
	}

	// straight exit
	if (!(HAS_EXIT_STRAIGHT(cc))) {
		outsideRegion = false;
		anchor = cc.crossingCenter + Point2d(offs, 0);
		normal = Point2d(0, -1);
		dist = distToWhite(anchor, normal, start, crossingExitMaxDist, outsideRegion);
		if (dist > minDist) {
			exits |= EXIT_STRAIGHT;
			if (prefs.debugCrossings) {
				cv::line(m_TopDown, anchor + normal * start, anchor + normal * dist, CV_RGB(0, 0, 255), 2, CV_AA);
				cv::circle(m_TopDown, anchor + normal * minDist, 2, CV_RGB(0, 0, 255), 2, CV_AA);
				LOG_INFO("Straight exit from image");
			}
		} else {
			cc.blockedExits |= EXIT_STRAIGHT;
		}
	}

	// left exit
	if (!(HAS_EXIT_LEFT(cc))) {
		outsideRegion = false;
		anchor = cc.crossingCenter + Point2d(0, -offs);
		normal = Point2d(-1, 0);
		dist = distToWhite(anchor, normal, start, crossingExitMaxDist, outsideRegion);
		if (dist > minDist) {
			exits |= EXIT_LEFT;
			if (prefs.debugCrossings) {
				cv::line(m_TopDown, anchor + normal * start, anchor + normal * dist, CV_RGB(0, 0, 255), 2, CV_AA);
				cv::circle(m_TopDown, anchor + normal * minDist, 2, CV_RGB(0, 0, 255), 2, CV_AA);
				LOG_INFO("Left exit from image");
			}
		} else {
			cc.blockedExits |= EXIT_LEFT;
		}
	}
	return exits;
}

tResult ProcessingThread::findParkingSlot()
{
	if (!prefs.pParking) {
		RETURN_NOERROR;
	}

	bool shouldDraw = imageProcessing->shouldOutputTopDown();
	bool outside = false;
	double sDist = prefs.pParkingSearchOffset / prefs.meterPerPixel;
	for (size_t i = 0; i < 4 && i < right.size() - 1; ++i) {
		Point2d laneP = right[i];
		Point2d n = Point2d(right[i + 1]) - laneP;
		n /= norm(n);
		Point2d nRot(-n.y, n.x);
		Point2d centerP = laneP - laneDistToPath * nRot;
		Point2d searchP = laneP + nRot * sDist;
		if (shouldDraw) {
			arrowedLine(m_TopDown, searchP, searchP + 20 * n, CV_RGB(255, 255, 0), 1, CV_AA);
		}
		outside = false;
		double dist = distToWhite(searchP, n, 1, 50, outside);
		imageProcessing->logger.log() << laneP << searchP << n << " " << dist << " " << outside << endl;
		if (!outside && dist < 50) {
			Point2d pPoint = centerP + dist * n;
			if (shouldDraw) {
				cv::circle(m_TopDown, pPoint, 5, CV_RGB(255, 255, 255), 2);
				cv::circle(m_TopDown, searchP + dist * n, 5, CV_RGB(255, 255, 255), 2);
			}
			Point2d worldPoint = imgToWorld(pPoint);
			ParkingSearchRequest r;
			r.dist = norm(worldPoint - Point2d(currentPose.x, currentPose.y));
			r.type = prefs.pType;
			imageProcessing->setParking(false);
			imageProcessing->logger.log() << "Parking line found: " << worldPoint << endl;
			return imageProcessing->transmitParkingData(r);
		}
	}
	RETURN_NOERROR;
}

int ProcessingThread::distToWhite(const cv::Point2d &anchor, const Point2d &normal, int start, int max, bool &outsideImageRegion) const
{
	Point2d n = normal / norm(normal);
	int i = start;
	for (; i < max; ++i) {
		Point2i p = anchor + n * static_cast<double>(i);
		if (!insideImageRegion(p)) {
			outsideImageRegion = true;
			break;
		}
		uchar val = m_matGreyThreshUncut.at<uint8_t>(p);
		if (val > 0) {
			outsideImageRegion = false;
			return i;
		}
	}
	outsideImageRegion = true;
	return i;
}

bool ProcessingThread::containsLanes(Mat area) const
{
	int nonzero = countNonZero(area);
	return nonzero > 0;
}

bool ProcessingThread::insideImageRegion(const Point &p) const
{
		if (p.y <= 2 || p.y >= 478) {
			// touches top or bottom of image
			return false;
		}
		if (p.x <= 2 || p.x >= 638) {
			// touches side of image
			return false;
		}
		if(pointPolygonTest(prefs.validImageRegion, p, true) < 5) {
			// Skip points that touch the image border
			return false;
		}
		return true;
}

bool ProcessingThread::insideCrossingRegion(const Point &p) const
{
	return pointPolygonTest(validCrossingRegion, p, false) > 0;
}

void ProcessingThread::extractCurves(vector<ProcessingThread::CurveDescription> &curves, const vector<ProcessingThread::Feature *> &features)
{
	for (size_t k = 0; k < features.size(); ++k) {
		if (features[k]->type != RightLaneCurve && features[k]->type != LeftLaneCurve) {
			continue;
		}
		if (features[k]->curveCenter.y < 200) {
			// Skip stuff that's totally outside the frame
			continue;
		}
		int idx = -1;
		if ((idx = containsCurve(curves, *features[k])) >= 0) {
			curves[idx].center += (Point2i(features[k]->curveCenter) -curves[idx].center) / 2.0;
		} else {
			CurveDescription c;
			c.center = features[k]->curveCenter;
			c.isBig = features[k]->radiusEstimate > 2.5 / prefs.meterPerPixel ||
					(features[k]->radiusEstimate > 1.5 / prefs.meterPerPixel &&
					 ((features[k]->type == RightLaneCurve && features[k]->vecToCenter.x < 0) || (features[k]->type == LeftLaneCurve && features[k]->vecToCenter.x > 0)));
			curves.push_back(c);
			if (prefs.debugPrintoutEnabled) {
				LOG_INFO(cString::Format("Added curve big %d right %d vectocenter %g", c.isBig, features[k]->type == RightLaneCurve, features[k]->vecToCenter.x));
			}
		}
	}
}

bool ProcessingThread::pointOnStreet(const Point &p) const
{
	double dist = 0;
	if (pointToSideOf(right, p, true, &dist)) {
		if (dist < l1m) {
			return true;
		}
	}
	if (pointToSideOf(left, p, false, &dist)) {
		if (dist < l1m) {
			return true;
		}
	}
	return false;
}

void ProcessingThread::updateOldLanes()
{
	transformLineToImg(worldLaneRight, oldRight);
	transformLineToImg(worldLaneLeft, oldLeft);
	transformLineToImg(worldLaneCenter, oldCenter);
	if (imageProcessing->shouldOutputTopDown()) {
		drawLine(oldRight, CV_RGB(128, 0, 0));
		drawLine(oldLeft, CV_RGB(128, 0, 0));
		drawLine(oldCenter, CV_RGB(128, 0, 0));
	}

}

void ProcessingThread::checkLaneReset()
{
	resetMutex.lock();
	if (!oldRight.empty() && oldRight.front().y < 300) {
		shouldResetRight = true;
	}
	if (shouldResetRight) {
		oldRight.clear();
		shouldResetRight = false;
	}
	if (shouldResetLeft) {
		oldLeft.clear();
		shouldResetLeft = false;
	}
	resetMutex.unlock();
}

void ProcessingThread::generateAndSendLaneData()
{
	transformLineToWorld(right, worldLaneRight);
	transformLineToWorld(left, worldLaneLeft);
	transformLineToWorld(center, worldLaneCenter);
	LaneData d;
	d.leftLane = worldLaneLeft;
	d.rightLane = worldLaneRight;
	d.centerLane = worldLaneCenter;
	imageProcessing->transmitLaneData(d);
	if (prefs.debugDrivingPath) {
		updateDrivingPath(d);
	}
}

void ProcessingThread::checkCenterFeature(ProcessingThread::Feature &f)
{
	if (isCenterLine(f) && fabs(f.rect.angle) < 20) {
		f.type = Center;
		centerLane.push_back(&f);
	}
}

void ProcessingThread::debugDrawFeatures()
{
	for (size_t i = 0; i < currFeatures.size(); ++i) {
		Point2f rectPoints[4];
		currFeatures[i].rect.points(rectPoints);
		for (size_t k = 0; k < 4; ++k) {
			if (currFeatures[i].type == Center) {
				line(m_TopDown, rectPoints[k], rectPoints[(k + 1) % 4], CV_RGB(0, 255, 0), 1, CV_AA);
			} else if (currFeatures[i].type == RightLaneCurve) {
				line(m_TopDown, rectPoints[k], rectPoints[(k + 1) % 4], CV_RGB(255, 0, 0), 1, CV_AA);
			} else if (currFeatures[i].type == LeftLaneCurve) {
				line(m_TopDown, rectPoints[k], rectPoints[(k + 1) % 4], CV_RGB(0, 0, 255), 1, CV_AA);
			} else {
				line(m_Drawing, rectPoints[k], rectPoints[(k + 1) % 4], Scalar(255), 1, CV_AA);
			}
		}
	}
}

void ProcessingThread::drawDrivingPath()
{
	const vector<Point2d> *worldPath = path.getPath();
	//polylines(m_TopDown, imgPath, false, CV_RGB(128, 0, 128), 1, CV_AA);
	transformLineToImg(*worldPath, imgPath);
	vector<Point2i> smoothedImagePath;
	smoothCurve((imgPath), smoothedImagePath, 5, 0);
	//polylines(m_TopDown, imgPath, false, CV_RGB(255, 0, 255), 1, CV_AA);
	drawFadedLine(m_TopDown, imgPath, CV_RGB(0, 0, 0), CV_RGB(255, 255, 255));
	//polylines(m_TopDown, smoothedImagePath, false, CV_RGB(255, 255, 255), 1, CV_AA);
}

void ProcessingThread::debugDrawCrossingExit(const ControllerCommand &c, Scalar col)
{
	Point2d p(c.pose.x, c.pose.y);
	Point2i imgP = worldToImg(p);
	Point2d cmdD = getOdometryDirection(c.pose);
	Point2i imgN = worldToImg(p + cmdD);
	cv::arrowedLine(m_TopDown, imgP, imgN, col);
}

void ProcessingThread::updateDrivingPath(const LaneData &l)
{
	// DEBUG
	path.processLaneData(l, currentPose, this, &imageProcessing->logger);
	ControllerCommand c = path.getNextCommand(currentPose);
	double t = (-90 - (c.pose.theta - currentPose.theta)) * M_PI / 180.0;
	Point2d p(c.pose.x, c.pose.y);
	Point2d pp = worldToImg(p);
	Point2d dir(cos(t), sin(t));
	arrowedLine(m_TopDown, pp, pp + dir * 20, CV_RGB(255, 0, 255), 2, CV_AA);
	if (path.empty()) {
		path.resetPath(currentPose);
	}
}

Point2d ProcessingThread::makeStopLineCrossingNormal(const Point2d &anchor, const Point2d &l2back) const
{
	Point2d n = anchor - l2back;
	n /= norm(n);
	return n * l50cm;
}

bool ProcessingThread::hasStopLine(const std::vector<Point2i> &closeLine, const std::vector<Point2i> &centerLine, const std::vector<Point2i> &farLine, const Point2d &anchor)
{
	if (closeLine.size() > 2) {
		Vec4f closeLineFit;
		fitLine(closeLine, closeLineFit, CV_DIST_L2, 0, .01, .01);
		if (pointDistToLine(anchor, closeLineFit) > l10cm) {
			return true;
		}
	} else if (centerLine.size() > 2) {
		Vec4f centerLaneFit;
		fitLine(centerLine, centerLaneFit, CV_DIST_L2, 0, .01, .01);
		if (pointDistToLine(anchor, centerLaneFit) < l10cm) {
			return true;
		}
	} else if (farLine.size() > 2) {
		Vec4f farLineFit;
		fitLine(farLine, farLineFit, CV_DIST_L2, 0, .01, .01);
		if (pointDistToLine(anchor, farLineFit) < l60cm) {
			return true;
		}
	}
	return false;
}

void ProcessingThread::setConstants()
{
	laneCenterLPix = prefs.laneCenterLength / prefs.meterPerPixel;
	laneCenterLDev = prefs.laneCenterDev / prefs.meterPerPixel; // Pixel
	crossingCornerL1Pix = prefs.crossingCornerLength1 / prefs.meterPerPixel;
	crossingCornerL2Pix = prefs.crossingCornerLength2 / prefs.meterPerPixel;
	laneDistToPath = .22 / prefs.meterPerPixel;
	l10cm = .1 / prefs.meterPerPixel;
	l60cm = .6 / prefs.meterPerPixel;
	l20cm = .2 / prefs.meterPerPixel;
	l50cm = .5 / prefs.meterPerPixel;
	l1m = 1.0 / prefs.meterPerPixel;
	crossingCenterDist = (1.27 / 2.0) / prefs.meterPerPixel;
	crossingExitMinFreeDist = .55 / prefs.meterPerPixel;
	crossingExitMaxDist = 1.0 / prefs.meterPerPixel;
}

void ProcessingThread::extractCrossings()
{
	vector<CrossingDetectionEvent> combinedCrossings;
	for (size_t i = 0; i < crossingCandidates.size(); ++i) {
		CrossingDetectionEvent c;
		Point2d currCenter = imgToWorld(crossingCandidates[i].crossingCenter);
		c.exits = crossingCandidates[i].exits;
		c.numDetections = 1;
		c.stopLine = crossingCandidates[i].stopLineOnTrack;
		c.center = currCenter;
		c.orientationVector = crossingCandidates[i].crossingNormal;
		mergeCrossings(combinedCrossings, c, .1);
	}

	for (size_t k = 0; k < combinedCrossings.size(); ++k) {
		CrossingDetectionEvent e = combinedCrossings[k];
		// Rotate normal to world
		e.orientationVector = normalToWorld(e.orientationVector, currentPose);
		if (prefs.debugCrossings) {
			imageProcessing->logger.log() << "Crossing at " << e.center << " with " << e.numDetections << " detections and " << hex << static_cast<int>(e.exits) << dec << " exits" << endl;
		}
		if (imageProcessing->shouldOutputTopDown()) {
			Point2i imgC = worldToImg(e.center);
			Point2i imgN = worldToImg(e.center + e.orientationVector);
			Point2i dirV(0, -1);
			dirV /= norm(dirV);
			circle(m_TopDown, imgC, 10, CV_RGB(255, 255, 255), 2, CV_AA);
			line(m_TopDown, imgC, imgN, CV_RGB(255, 255, 255), 2, CV_AA);
			if (e.stopLine) {
				Point2i a = imgC - l50cm * dirV;
				Point2i b = a + Point2i(50, 0);
				line(m_TopDown, a, b, CV_RGB(255, 255, 255), 2, CV_AA);
			}
			if (HAS_EXIT_RIGHT(e)) {
				arrowedLine(m_TopDown, imgC, imgC + 20 * dirV, CV_RGB(255, 255, 255));
			}
			if (HAS_EXIT_LEFT(e)) {
				arrowedLine(m_TopDown, imgC, imgC + Point2i(-20, 0), CV_RGB(255, 255, 255));
			}
			if (HAS_EXIT_RIGHT(e)) {
				arrowedLine(m_TopDown, imgC, imgC + Point2i(20, 0), CV_RGB(255, 255, 255));
			}
			if (HAS_EXIT_BACK(e)) {
				arrowedLine(m_TopDown, imgC, imgC - 20 * dirV, CV_RGB(255, 255, 255));
			}
		}
		if (prefs.debugCrossings) {
			debugDrawCrossingExit(getCrossingExit(e, EXIT_STRAIGHT), CV_RGB(255, 0, 0));
			debugDrawCrossingExit(getCrossingExit(e, EXIT_LEFT), CV_RGB(0, 255, 0));
			debugDrawCrossingExit(getCrossingExit(e, EXIT_RIGHT), CV_RGB(0, 0, 255));
			debugDrawCrossingExit(getCrossingStop(e), CV_RGB(255, 255, 255));
		}
		imageProcessing->transmitCrossingData(e);
	}
}

void ProcessingThread::fitCenterLine()
{
	if (center.size() >= 2) {
		Vec4f temp;
		fitLine(center, temp, CV_DIST_L2, 0, 0.1, 0.1);
		Point2d normal(temp[0], temp[1]);
		if (normal.y > 0) {
			temp[0] *= -1;
			temp[1] *= -1;
		}
		if (fabs(atan(normal.x / normal.y)) < M_PI / 16) {
			centerLineAge = 0;
			centerLine = temp;
			lastCenterPose = currentPose;
			centerLineAngleDelta = 0;
		} else {
			centerLineAge = 999;
		}
	} else {
		centerLineAge = 999;
	}
	if (centerLineAge < centerLineMaxAge && fabs(atan(centerLine[0] / centerLine[1])) < M_PI / 16.0) {
		Point2d anchor(centerLine[2], centerLine[3]);
		Point2d normal(centerLine[0], centerLine[1]);
		line(m_TopDown, anchor - 500 * normal, anchor + 500 * normal, CV_RGB(255, 0, 0), 2, CV_AA);
	}
}

void ProcessingThread::doLineFits(const vector<Point2i> &right)
{
	size_t n = right.size();
	size_t divider = n / 2;
	int depth = 4;
	double error = getLinearityError(right, 0, n - 1);
	double eta = 3;
	bool fromTop = false;
	if (error < 3 && error >= 0) {
		// done
		divider = n - 1;
	} else {
		double error1 = getLinearityError(right, 0, divider);
		double error2 = getLinearityError(right, divider, n - 1);
		fromTop = error1 < error2;
		if (fromTop) {
			while (n / depth > 1) {
				size_t diva = divider + n / depth;
				size_t divb = divider - n / depth;
				if (diva > n - 1 || diva < 0 || divb > n - 1 || divb < 0) {
					imageProcessing->logger.log() << "break " << diva << " " << divb << endl;
					break;
				}
				error1 = getLinearityError(right, divb, n - 1);
				error2 = getLinearityError(right, diva, n - 1);
				if (error1 >= 0 && error1 < eta) {
					divider = divb;
				} else {
					divider = diva;
				}
				depth *= 2;
			}
		} else {
			while (n / depth > 1) {
				size_t diva = divider + n / depth;
				size_t divb = divider - n / depth;
				if (diva > n - 1 || diva < 0 || divb > n - 1 || divb < 0) {
					imageProcessing->logger.log() << "break " << diva << " " << divb << endl;
					break;
				}
				error1 = getLinearityError(right, 0, diva);
				error2 = getLinearityError(right, 0, divb);
				if (error1 >= 0 && error1 < eta) {
					divider = diva;
				} else {
					divider = divb;
				}
				depth *= 2;
			}
		}
		/*
		vector<Point2i> curve(fromTop ? right.begin() : right.begin() + divider, right.begin() + (fromTop ? divider : n - 1));
		if (curve.size() >= 5) {
			cv::RotatedRect elypse = fitEllipse(curve);
			cv::ellipse(m_TopDown, elypse, CV_RGB(255, 255, 0), 2, CV_AA);
		}
		*/
	}
	imageProcessing->logger.log() << "fromTop " << fromTop << " n " << n << " divider " << divider << " start " << 0 + (fromTop ? divider : 0) << " end " << 0 + (fromTop ? n - 1 : divider) << endl;
	line(m_TopDown, *(right.begin() + (fromTop ? divider : 0)), *(right.begin() + (fromTop ? n - 1 : divider)), CV_RGB(255, 255, 255), 2, CV_AA);
}

Scalar cols[] = {CV_RGB(200, 0, 0), CV_RGB(0, 0, 200), CV_RGB(0, 200, 0), CV_RGB(200, 200, 0), CV_RGB(0, 200, 200)};

// Debug draw function for processlanedata
void ProcessingThread::drivingPathDebugDrawFunc(const vector<Point2d> &v, const vector<float> &w)
{
	vector<Point2i> temp;
	transformLineToImg(v, temp);
	Point2i center = getAvgPoint(temp, w);
	for (size_t i = 0; i < v.size(); ++i) {
		if (w[i] > 0) {
			circle(m_TopDown, temp[i], 2, cols[i], 1);
			line(m_TopDown, center, temp[i], cols[i], 1, CV_AA);
		}
	}
}

void ProcessingThread::drivingPathDebugDrawCircle(const Point2d &p, int r, Scalar col)
{
	Point2i imgP = worldToImg(p);
	cv::circle(m_TopDown, imgP, r, col);
}

// TODO get steering angle and adjust search starts for curves
void ProcessingThread::findLeftRightLanes()
{
	// If old data is bad, perform reset
	checkLaneReset();
	bool drawingEnabled = imageProcessing->shouldOutputTopDown();

	Point2i last(prefs.rightLaneSearchX, prefs.rightLaneSearchY);
	Point2d normal(1, 0);
	//right = getLaneEstimate(last, normal, oldRight);
	//right = extractLane(last, RightLaneSearch, oldRight);
	right = extractLane(last, RightLaneSearch, oldRight, prefs.debugLaneFinder && drawingEnabled);

	last = Point2i(prefs.leftLaneSearchX - static_cast<int>(.4 / prefs.meterPerPixel), prefs.leftLaneSearchY);
	if (drawingEnabled && prefs.debugLaneFinder) {
		cv::circle(m_TopDown, last, 2, CV_RGB(255, 0, 255), 1);
	}

	// Try setting the search start point from the current center lane
	Point2d leftStart = last;
	double minDist = 64;
	for (size_t i = 0; i < center.size(); ++i) {
		double dist = norm(center[i] - last);
		if (dist < minDist && center[i].y < 420) {
			minDist = dist;
			leftStart = center[i];
			leftStart.x -= laneDistToPath;
		}
	}
	for (int i = 0; i < 5; ++i) {
		Point2i p(leftStart.x - l20cm, leftStart.y - i *10);
		if (insideImageRegion(p)) {
			leftStart.y = p.y;
			break;
		}
	}
	if (drawingEnabled && prefs.debugLaneFinder) {
		cv::circle(m_TopDown, leftStart, 4, CV_RGB(255, 0, 255), 1);
	}
	normal.x = -1;
	//vector<Point2i> left = getLaneEstimate(last, normal, oldLeft);
	left = extractLane(leftStart, LeftLaneSearch, oldLeft, prefs.debugLaneFinder && drawingEnabled);
	if (left.size() < 2) {
		LOG_LANE("Trying left lane curve right");
		last = Point2i(prefs.leftLaneSearchX + prefs.leftLaneSearchCurveRightOffs.x - static_cast<int>(.4 / prefs.meterPerPixel), prefs.leftLaneSearchY + prefs.leftLaneSearchCurveRightOffs.y);
		Point2d n(-.5, -.5);
		n /= norm(n);
		left = extractLane(last, LeftLaneSearch, oldLeft, prefs.debugLaneFinder && drawingEnabled, &n);
	}
	if (left.size() < 2) {
		LOG_LANE("Trying left lane curve left");
		last = Point2i(prefs.leftLaneSearchX + prefs.leftLaneSearchCurveLeftOffs.x - static_cast<int>(.4 / prefs.meterPerPixel), prefs.leftLaneSearchY + prefs.leftLaneSearchCurveLeftOffs.y);
		Point2d n(-.5, .5);
		n /= norm(n);
		left = extractLane(last, LeftLaneSearch, oldLeft, prefs.debugLaneFinder && drawingEnabled, &n);
	}

	if (drawingEnabled) {
		drawLine(right, CV_RGB(255, 0, 0));
		drawLine(left, CV_RGB(255, 0, 0));
	}
}

std::vector<Point2i> ProcessingThread::getLaneEstimate(const Point2i &_start, const Point2d &_normal, const std::vector<Point2i> &oldLane, int stepsize, size_t num) const
{
	vector<Point2i> lane;
	Point2i last = _start;
	int maxLaneSearchDist = l50cm;
	for (size_t i = 0; i < oldLane.size(); ++i) {
		if (oldLane[i].y < 475 && oldLane[i].y > 460) {
			last = oldLane[i];
			last.x -= laneDistToPath;
			break;
		}
	}
	Point2i vec(0, -stepsize);
	Point2d normal = _normal;
	double lastAngle = -M_PI_2 + (currentPose.theta - lastPose.theta) * M_PI / 180.0;
	int blindCounter = 0;
	Point2i lastLanePoint(Point2d(last) + normal * laneDistToPath);

	bool outside = false;
	//cv::circle(m_TopDown, last, 2, CV_RGB(0, 0, 255), 2, CV_AA);
	// correct first point
	for (int i = 0; i < 2; ++i) {
		Point2d foo = last;
		double dist = distToWhite(foo, normal, 0, maxLaneSearchDist, outside);
		if (!outside) {
			last = foo + normal * (dist - laneDistToPath);
			lastLanePoint = foo + normal * dist;
			cv::circle(m_TopDown, last, 2, CV_RGB(0, 0, 255), 2, CV_AA);
			break;
		} else {
			last = foo - 5 * normal;
		}
	}
	if (outside) {
		cv::circle(m_TopDown, last, 2, CV_RGB(0, 255, 0), 2, CV_AA);
	}


	for (size_t i = 1; i < num; ++i) {
		Point2d curr = last + vec;
		cv::circle(m_TopDown, curr, 2, CV_RGB(0, 255, 255), 2, CV_AA);
		outside = false;
		double dist = distToWhite(curr, normal, 0, maxLaneSearchDist, outside);
		if (!outside) {
			// shift point 22 cm left of lane
			Point2i currLanePoint = curr + normal * dist;
			Point2i next(curr + normal * (dist - laneDistToPath));
			Point2i temp = currLanePoint - lastLanePoint;
			double angle = atan2(temp.y, temp.x);
			cv::line(m_TopDown, currLanePoint, lastLanePoint, CV_RGB(0,255,0), 1, CV_AA);
			if (i > 1 && !isParallel(angle, lastAngle, M_PI_2 / 2)) {
				cv::line(m_TopDown, last, curr, CV_RGB(0, 0, 255), 1, CV_AA);
				last = curr;
				++blindCounter;
				if (i > 10 && blindCounter > 5) {
					cv::circle(m_TopDown, curr, 5, CV_RGB(255, 255, 128), 3, CV_AA);
					break;
				}
				continue;
			}
			if (norm(temp) > abs(4 * stepsize) && blindCounter < 2) {
				cv::line(m_TopDown, last, curr, CV_RGB(0, 255, 255), 1, CV_AA);
				cv::circle(m_TopDown, curr, 5, CV_RGB(255, 255, 255), 3, CV_AA);
				break;
			}
			lastLanePoint = currLanePoint;
			blindCounter = 0;
			lastAngle = angle;
			double l = norm(temp);
			vec.x = static_cast<int>(stepsize * temp.x / l);
			vec.y = static_cast<int>(stepsize * temp.y / l);
			lane.push_back(currLanePoint);
			cv::line(m_TopDown, last, next, CV_RGB(0, 255, 255), 1, CV_AA);
			cv::line(m_TopDown, curr, lane.back(), CV_RGB(0, 0, 255), 1, CV_AA);
			last = next;
		} else {
			cv::line(m_TopDown, last, curr, CV_RGB(255, 0, 0), 1, CV_AA);
			last = curr;
			++blindCounter;
		}
		if (i > 10 && (!insideImageRegion(last) || blindCounter > 5)) {
			cv::circle(m_TopDown, curr, 5, CV_RGB(255, blindCounter <= 5 ? 0 : 255, blindCounter > 5 ? 128 : 0), 3, CV_AA);
			break;
		}
	}
	return lane;
}

std::vector<Point2i> ProcessingThread::extractLane(const Point2i &_start, const ProcessingThread::LaneSearchType type, const std::vector<Point2i> &oldLane, bool debugDraw, Point2d *defSearchNormal)
{
	for (size_t i = 2; i < oldLane.size(); ++i) {
		Point2d diff1 = oldLane[i] - oldLane[i - 1];
		Point2d diff2 = oldLane[i - 1] - oldLane[i - 2];
		double angleDiff = fixAngle(atan2(diff1.y,diff1.x) - atan2(diff2.y, diff2.x));
	}
	searchPointDist = laneDistToPath;
	double maxDev = prefs.laneDetectorMaxDev;
	vector<Point2i> lane;
	// Current point in center of lane from which to search
	Point2i currSearchPoint = _start;
	// Direction in which to search
	// TODO seed from last lane/wheel angle
	Point2d currSearchNormal(type == RightLaneSearch ? 1 : -1, 0);
	if (defSearchNormal) {
		currSearchNormal = *defSearchNormal;
	}
	if (debugDraw) {
		cv::arrowedLine(m_TopDown, currSearchPoint, Point2d(currSearchPoint) + searchPointDist * currSearchNormal, CV_RGB(255, 0, 255), 1);
	}

	size_t pos = 0;
	while (pos < oldLane.size() && !insideImageRegion(oldLane[pos])) {
		++pos;
	}

	int guesses = 0;
	int confirmeds = 0;

	bool done = false;
	bool first = true;
	int badData = 0;
	if (prefs.debugLaneFinder) {
		imageProcessing->logger.log() << "skipped " << pos << " of " << oldLane.size() <<  " points" << endl;
	}
	for (; pos < oldLane.size(); ++pos) {
		//imageProcessing->logger.log() << pos << endl;
		// If we're outisde the region, just keep the rest of the old lane
		if (!insideImageRegion(oldLane[pos])) {
			if (pointContinuesLane(lane, oldLane[pos])) {
				lane.push_back(oldLane[pos]);
				++guesses;
				if (debugDraw) {
					cv::circle(m_TopDown, lane.back(), 2, CV_RGB(255, 0, 0), 1);
				}
				done = true;
				continue;
			} else {
				LOG_LANE("No more points to add from old lane (image border hit), done");
				done = true;
				break;
			}
		}

		// Take search point perpendicular to current point in old lane
		if (pos < oldLane.size() - 1) {
			getNextSearchPoint(oldLane[pos], oldLane[pos + 1] - oldLane[pos], type, currSearchPoint, currSearchNormal, first, debugDraw);
		} else if (pos > 0){
			getNextSearchPoint(oldLane[pos], oldLane[pos] - oldLane[pos - 1], type, currSearchPoint, currSearchNormal, first, debugDraw);
		} else {
			getNextSearchPoint(oldLane[pos], getExtrapolatedPoint(oldLane, currSearchPoint - Point2i(currSearchNormal * searchPointDist) - oldLane[pos], currSearchNormal), type, currSearchPoint, currSearchNormal, first, debugDraw);
		}
		first = false;

		bool outside = false;
		double dist = distToWhite(currSearchPoint, currSearchNormal, 0, 2 * searchPointDist, outside);
		if (outside) {
			// No new point found
			if (!pointContinuesLane(lane, oldLane[pos])) {
				// Point from old lane doesn't fit new lane so we extrapolate
				LOG_LANE("Old point failed lane check, inserting dummy");
				++badData;
				if (badData > 3) {
					LOG_LANE("Bad data, giving up");
					break;
				}
				Point2i temp = getExtrapolatedPoint(lane, currSearchPoint, currSearchNormal);
				if (norm(temp - oldLane[pos]) > maxDev) {
					LOG_LANE("Dist getting too big for extrapolation, aborting");
					break;
				}
				if (debugDraw) {
					cv::circle(m_TopDown, lane.back(), 2, CV_RGB(0, 0, 255), 1);
				}
			} else {
				// Old point was ok, use it
				lane.push_back(oldLane[pos]);
				++guesses;
				badData = 0;
				if (debugDraw) {
					cv::circle(m_TopDown, lane.back(), 2, CV_RGB(255, 0, 0), 1);
				}
			}
		} else {
			// We found a point within the tolerance
			Point2i currPoint(currSearchPoint.x + dist * currSearchNormal.x, currSearchPoint.y + dist * currSearchNormal.y);
			if (norm(currPoint - oldLane[pos]) > maxDev) {
				// but it's too far away, so check if the old point was good
				if (!pointContinuesLane(lane, oldLane[pos])) {
					// the old point sucks
					LOG_LANE("Old point failed lane check, inserting dummy");
					++badData;
					if (badData > 3) {
						LOG_LANE("Bad data, giving up");
						break;
					}
					Point2i temp = getExtrapolatedPoint(lane, currSearchPoint, currSearchNormal);
					if (norm(temp - oldLane[pos]) > maxDev) {
						LOG_LANE("Dist getting too big for extrapolation, aborting");
						break;
					}
					lane.push_back(getAvgPoint(oldLane[pos], temp));
					++guesses;
					if (debugDraw) {
						cv::circle(m_TopDown, lane.back(), 4, CV_RGB(0, 0, 255), 1);
					}
				} else {
					// The old point doesn't suck
					lane.push_back(oldLane[pos]);
					++guesses;
					badData = 0;
					if (debugDraw) {
						cv::circle(m_TopDown, lane.back(), 4, CV_RGB(255, 0, 0), 1);
					}
				}
				if (debugDraw) {
					cv::circle(m_TopDown, currPoint, 2, CV_RGB(0, 255, 0), 1);
				}
			} else {
				// New point is close enough to old lane
				if (!pointContinuesLane(lane, currPoint)) {
					// But the angle sucks
					LOG_LANE("New point failed angle check, inserting dummy");
					++badData;
					if (badData > 3) {
						LOG_LANE("Bad data, giving up");
						break;
					}
					Point2i temp = getExtrapolatedPoint(lane, currSearchPoint, currSearchNormal);
					if (norm(temp - oldLane[pos]) > maxDev) {
						LOG_LANE("Dist getting too big for extrapolation, aborting");
						break;
					}
					lane.push_back(getAvgPoint(oldLane[pos], temp));
					++guesses;
					//lane.push_back(temp);
					if (debugDraw) {
						cv::circle(m_TopDown, lane.back(), 4, CV_RGB(0, 0, 255), 1);
						cv::circle(m_TopDown, currPoint, 4, CV_RGB(255, 255, 255), 1);
						cv::line(m_TopDown, oldLane[pos], temp, CV_RGB(255, 255, 0), 1, CV_AA);
					}
				} else {
					// New point is good
					lane.push_back(currPoint);
					++confirmeds;
					if (debugDraw) {
						cv::circle(m_TopDown, lane.back(), 4, CV_RGB(0, 255, 0), 1);
					}
					badData = 0;
				}
			}
		}
	}

	if (!lane.empty()) {
		if (static_cast<double>(confirmeds) / lane.size() < .3) {
			resetLaneTracking(type);
		}
	}

	if (!lane.empty() && !insideImageRegion(lane.back())) {
		LOG_LANE("Aborting because last point is outside the array");
		return lane;
	}
	if (done) {
		LOG_LANE("Aborting because done");
		return lane;
	}
	badData = 0;

	if (debugDraw && !lane.empty()) {
		cv::circle(m_TopDown, lane.back(), 8, CV_RGB(128, 128, 128), 1, CV_AA);
	}
	LOG_LANE("Searching new points");

	// old lane ended before image did, start extrapolating
	do {
		Point2i estPoint = getExtrapolatedPoint(lane, currSearchPoint, currSearchNormal);
		if (debugDraw) {
			cv::circle(m_TopDown, estPoint, 2, CV_RGB(0, 0, 128), 1);
		}
		bool insideRegion = insideImageRegion(estPoint);
		if (!insideRegion || !pointContinuesLane(lane, estPoint)) {
			if (!insideRegion) {
				if (prefs.debugLaneFinder) {
					imageProcessing->logger.log() << "Image end reached. stopping: " << estPoint << endl;
				}
			}
			break;
		}
		if (!lane.empty()) {
			getNextSearchPoint(estPoint, estPoint - lane.back(), type, currSearchPoint, currSearchNormal, false, debugDraw);
		} else {
			LOG_LANE("No search point");
			if (defSearchNormal) {
				LOG_LANE("Forcing default search");
				currSearchNormal = *defSearchNormal;
				currSearchPoint = _start;
			}
		}
		bool outside = false;
		double dist = distToWhite(currSearchPoint, currSearchNormal, 0, 2 * searchPointDist, outside);
		if (outside) {
			if (prefs.debugLaneFinder) {
				imageProcessing->logger.log() << "Aborting (outside) " << currSearchPoint << " dist " << dist << endl;
			}
			break;
		}
		Point2i currPoint(currSearchPoint.x + dist * currSearchNormal.x, currSearchPoint.y + dist * currSearchNormal.y);
		if (norm(estPoint - currPoint) > maxDev) {
			if (!pointContinuesLane(lane, estPoint)) {
				LOG_LANE("Est point failed angle check, aborting");
				cv::circle(m_TopDown, estPoint, 4, CV_RGB(255, 255, 255), 1);
				break;
			}
			lane.push_back(estPoint);
			++guesses;
			if (debugDraw) {
				cv::circle(m_TopDown, currPoint, 2, CV_RGB(0, 255, 0), 1);
				cv::circle(m_TopDown, lane.back(), 4, CV_RGB(0, 0, 255), 1);
			}
			break;
		} else {
			if (!pointContinuesLane(lane, currPoint)) {
				LOG_LANE("Lane point failed angle check");
				if (debugDraw) {
					cv::circle(m_TopDown, currPoint, 4, CV_RGB(255, 255, 255), 1);
					cv::circle(m_TopDown, estPoint, 4, CV_RGB(0, 0, 255), 1);
				}
				lane.push_back(estPoint);
				++guesses;
				++badData;
				if (badData > 3) {
					LOG_LANE("Bad data, giving up");
					break;
				}
			} else {
				badData = 0;
				lane.push_back(currPoint);
				++confirmeds;
				if (debugDraw) {
					cv::circle(m_TopDown, lane.back(), 4, CV_RGB(0, 255, 255), 1);
				}
			}
		}
	} while (insideImageRegion(lane.back()));

	if (!lane.empty()) {
		if (static_cast<double>(confirmeds) / lane.size() < .3) {
			resetLaneTracking(type);
		}
	}

	return lane;
}

vector<Point2i> ProcessingThread::extractCenterLane()
{
	bool drawingEnabled = imageProcessing->shouldOutputTopDown();
	std::sort(centerLane.begin(), centerLane.end(), &featureLowerThan);
	vector<Point2i> center;
	Point2i last(320 - laneDistToPath, 480);
	//center.push_back(last);
	double lastRot = 0;
	for (size_t i = 0; i < centerLane.size(); ++i) {
		bool distFail = false;
		if ((distFail = fabs(centerLane[i]->rect.center.x - last.x) > 50) ||
				fabs(centerLane[i]->rect.angle - lastRot) > 35) {
			if (drawingEnabled) {
				cv::circle(m_TopDown, centerLane[i]->rect.center, 3, distFail ? CV_RGB(0,255,0) : CV_RGB(0, 0, 255), 1, CV_AA);
			}
			continue;
		}
		// Allow 200 pixels for first center line, otherwise 100 between them.
		if (norm(Point2i(centerLane[i]->rect.center) - last) > (center.empty() ? 200 : 100)) {
			if (drawingEnabled) {
				cv::circle(m_TopDown, centerLane[i]->rect.center, 3, CV_RGB(255, 0, 0), 1, CV_AA);
			}
			continue;
		}
		if (drawingEnabled) {
			cv::line(m_TopDown, last, centerLane[i]->rect.center, CV_RGB(255, 0, 0), 1, CV_AA);
		}
		center.push_back(centerLane[i]->rect.center);
		last = centerLane[i]->rect.center;
	}
	return center;
}

void ProcessingThread::getNextSearchPoint(const Point2d &lanePoint, const Point2d &laneTangent, LaneSearchType type, cv::Point2i &currSearchPoint, cv::Point2d &currSearchNormal, bool noCheck, bool debugDraw)
{
	double oldAngle = atan2(currSearchNormal.y, currSearchNormal.x);
	Point2d oldNormal = currSearchNormal;
	Point2i oldPoint = currSearchPoint;
	currSearchPoint = lanePoint;
	currSearchNormal = laneTangent;
	currSearchNormal /= norm(currSearchNormal);
	currSearchNormal = Point2d(-currSearchNormal.y, currSearchNormal.x);
	if (type == LeftLaneSearch) {
		currSearchNormal = - currSearchNormal;
	}
	currSearchPoint -= Point2i(searchPointDist * currSearchNormal);
	// TODO fix this bs for curves
	if (!noCheck && (norm(currSearchPoint - oldPoint) > 2 * prefs.laneExtrapolationDist
					 || fabs(fixAngle(atan2(currSearchNormal.y,currSearchNormal.x) - oldAngle)) > M_PI_4 / 2)) {
		if (debugDraw) {
			cv::arrowedLine(m_TopDown, currSearchPoint, Point2d(currSearchPoint) + 20 * currSearchNormal, CV_RGB(255, 0, 0), 1);
		}
		Point2i badPoint = currSearchPoint;
		currSearchPoint = oldPoint;
		if (type == RightLaneSearch) {
			currSearchPoint += Point2i(prefs.laneExtrapolationDist * oldNormal.y, prefs.laneExtrapolationDist * -oldNormal.x);
		} else {
			currSearchPoint += Point2i(prefs.laneExtrapolationDist * -oldNormal.y, prefs.laneExtrapolationDist * oldNormal.x);
		}
		currSearchNormal = oldNormal;
		if (debugDraw) {
			cv::arrowedLine(m_TopDown, currSearchPoint, Point2d(currSearchPoint) + 20 * currSearchNormal, CV_RGB(0, 255, 255), 1);
			cv::line(m_TopDown, badPoint, currSearchPoint, CV_RGB(255, 255, 255), 1, CV_AA);
			cv::line(m_TopDown, oldPoint, currSearchPoint, CV_RGB(128, 128, 128), 1, CV_AA);
		}
	} else if (debugDraw) {
		cv::arrowedLine(m_TopDown, currSearchPoint, Point2d(currSearchPoint) + 20 * currSearchNormal, CV_RGB(255, 255, 0), 1);
	}
}

Point2d ProcessingThread::getExtrapolatedPoint(const std::vector<Point2i> &lane, const Point2d &currSearchPoint, const Point2d &currSearchNormal)
{
	double extrapolationDist = prefs.laneExtrapolationDist;
	Point2i defPoint = currSearchPoint + searchPointDist * currSearchNormal;
	bool rightLane = currSearchNormal.x > 0;
	Point2d defDir (rightLane ? currSearchNormal.y : -currSearchNormal.y, rightLane ? -currSearchNormal.x : currSearchNormal.x);
	return extrapolateLine(lane, lane.size(), extrapolationDist, defPoint, defDir);
}

double ProcessingThread::distFromCarSqrt(const Point &p)
{
	return (p.x - 320) * (p.x - 320) + p.y * p.y;
}

const int maxCurveCenterOffset = 4000;

int ProcessingThread::containsCurve(const vector<ProcessingThread::CurveDescription> &curves, const ProcessingThread::Feature &f)
{
	for (size_t k = 0; k < curves.size(); ++k) {
		if (distSquare(f.curveCenter, curves[k].center) < maxCurveCenterOffset) {
			return k;
		}
	}
	return -1;
}

void ProcessingThread::getDistanceFromSide(const vector<Point> &contour, const Feature &f, double *inner, double *outer, cv::Point2f *_normal)
{
	Point2f normal;
	if (!_normal) {
		double angRad = M_PI * f.rect.angle / 180.0;
		normal.x = cos(angRad);
		normal.y = sin(angRad);
		if (angRad > 0) {
			normal *= -1;
		}
	} else {
		normal = *_normal;
	}
	if (outer) {
		*outer = f.shortSide + pointPolygonTest(contour, f.rect.center + normal * f.shortSide / 2.0, true);
	}
	if (inner) {
		*inner = -pointPolygonTest(contour, f.rect.center - normal * f.shortSide / 2.0, true);
	}
}

void ProcessingThread::drawLine(const std::vector<Point> &line, cv::Scalar colour, int thickness, int lineType)
{
	for (size_t i = 1; i < line.size(); ++i) {
		cv::line(m_TopDown, line[i - 1], line[i], colour, thickness, lineType);
	}
}

void ProcessingThread::smoothCurve(const std::vector<cv::Point2i> &curve, std::vector<cv::Point2i> &output, int smoothingSize, float smoothingShape)
{
	if (smoothingSize == 0) {
		LOG_ERROR("Smoothing size must be > 0");
		return;
	}
	int n = curve.size();
	smoothingSize = clamp(smoothingSize, 0, n);
	smoothingShape = clamp(smoothingShape, 0.0f, 1.0f);

	// Precompute weights and normalization
	vector<float> weights;
	weights.resize(smoothingSize);
	// side weights
	for (int i = 1; i < smoothingSize; ++i) {
		// remap value to [1, smoothingShape)
		weights[i] = (static_cast<float>(i) / static_cast<float>(smoothingSize) * (smoothingShape - 1) + 1);
	}
	vector<Point2f> points;
	points.resize(curve.size());
	output.resize(curve.size());
	for (size_t i = 0; i < curve.size(); ++i) {
		points[i] = Point2f(curve[i]);
	}


	for (int i = 0; i < n; ++i) {
		float sum = 1; // center weight
		for (int j = 1; j < smoothingSize; ++j) {
			Point2f cur;
			int leftPosition = i - j;
			int rightPosition = i + j;
			if (leftPosition >= 0) {
				cur += Point2f(curve[leftPosition]);
				sum += weights[j];
			}
			if (rightPosition < n) {
				cur += Point2f(curve[rightPosition]);
				sum += weights[j];
			}
			points[i] += cur * weights[j];
		}
		output[i] = points[i] / sum;
	}
}

void ProcessingThread::updateTransform()
{
	double sinth = sin(currentPose.theta * M_PI / 180.0);
	double costh = cos(currentPose.theta * M_PI / 180.0);
	a1 = currentPose.x + (prefs.meterToTopDownCenter + 240 * prefs.meterPerPixel) * costh - 320 * prefs.meterPerPixel * sinth;
	b1 = prefs.meterPerPixel * sinth; // * u
	c1 = -prefs.meterPerPixel * costh; // * v

	a2 = currentPose.y + 320 * prefs.meterPerPixel * costh + (prefs.meterToTopDownCenter + 240 * prefs.meterPerPixel) * sinth;
	b2 = c1;
	c2 = -b1;

	// inverse transform
	a3 = (320 * prefs.meterPerPixel + currentPose.y * costh - currentPose.x * sinth) / prefs.meterPerPixel;
	b3 = sinth / prefs.meterPerPixel;
	c3 = -costh / prefs.meterPerPixel;

	a4 = (prefs.meterToTopDownCenter + 240 * prefs.meterPerPixel + currentPose.x * costh + currentPose.y * sinth) / prefs.meterPerPixel;
	b4 = c3;
	c4 = -b3;
}

Point2d ProcessingThread::imgToWorld(const Point &p) const
{
	return Point2d(a1 + b1 * p.x + c1 * p.y, a2 + b2 * p.x + c2 * p.y);
}

Point2i ProcessingThread::worldToImg(const Point2d &p) const
{
	return Point2i(static_cast<int>(round(a3 + b3 * p.x + c3 * p.y)), static_cast<int>(round(a4 + b4 * p.x + c4 * p.y)));
}

void ProcessingThread::transformLineToWorld(const std::vector<Point> &input, std::vector<Point2d> &output) const
{
	output.resize(input.size());
	for (size_t i = 0; i < input.size(); ++i) {
		output[i] = imgToWorld(input[i]);
	}
}

void ProcessingThread::transformLineToImg(const std::vector<Point2d> &input, std::vector<Point2i> &output) const
{
	output.resize(input.size());
	for (size_t i = 0; i < input.size(); ++i) {
		output[i] = worldToImg(input[i]);
	}
}

void ProcessingThread::testTransforms() const
{
	Point2i testPoint(10, 10);
	Point2d world = imgToWorld(testPoint);
	Point2i result = worldToImg(world);
	imageProcessing->logger.log() << testPoint.x << ", " << testPoint.y << " -> " << world.x << ", " << world.y << " -> " << result.x << ", " << result.y << endl;
}
