#include "SR_SignMapping.h"
#include "../misc/PinWrapper.h"
#include <map_data.h>
// Create filter shell

#define PROP_ALPHA "Camera::Camera Angle [deg]"
#define PROP_CAM_OFFSET_Z "Camera::Camera height [m]"
#define PROP_CAM_OFFSET_X "Camera::Camera distance [m]"
#define PROP_CAM_OFFSET_Y "Camera::Camera side offset [m]"

#undef LOG_INFO
#define LOG_INFO(str) logger.GLogTrap((str));

ADTF_FILTER_PLUGIN("SpaceRacer Sign Mapping", OID_SR_SIGNMAPPING, SignMapping)

SignMapping::SignMapping(const tChar *info) : cFilter(info),logger(OID_SR_SIGNMAPPING)
{
	m_firstTime = tTrue;


	//camAngle = 24.5;
	camAngle = 0.0;
	camOffsetX = .3;
	camOffsetZ = .21;
	camOffsetY = .025;

	SetPropertyFloat(PROP_ALPHA, camAngle);
	SetPropertyBool(PROP_ALPHA NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyFloat(PROP_CAM_OFFSET_Y, camOffsetY);
	SetPropertyBool(PROP_CAM_OFFSET_Y NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyFloat(PROP_CAM_OFFSET_Z, camOffsetZ);
	SetPropertyBool(PROP_CAM_OFFSET_Z NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyFloat(PROP_CAM_OFFSET_X, camOffsetX);
	SetPropertyBool(PROP_CAM_OFFSET_X NSSUBPROP_ISCHANGEABLE, tTrue);

}

SignMapping::~SignMapping()
{

}


tResult SignMapping::Init(adtf::cFilter::tInitStage stage, ucom::IException **__exception_ptr)
{
	// call base implementation
	RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))

			if (stage == StageFirst) {
		RETURN_IF_FAILED(signInput.Create("Road_Sign", new cMediaType(0, 0, 0, "tRoadSignExt"), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&signInput));

		REGISTER_MEDIA_PIN(posePin, MEDIA_TYPE_ODOMETRY, MEDIA_SUBTYPE_ODOMETRY_POSE, "Current_Pose");
		REGISTER_MEDIA_PIN(signOutput, MEDIA_TYPE_MAPDATA, MEDIA_SUBTYPE_MAPDATA_SIGN, "Sign_output");

	} else if (stage == StageNormal) {
		currentPose.x=0;
		currentPose.y=0;
		currentPose.theta=0;
		currentPose.speed=0;
		camAngle= GetPropertyFloat(PROP_ALPHA, camAngle);
		camOffsetY = GetPropertyFloat(PROP_CAM_OFFSET_Y, camOffsetY);
		camOffsetZ = GetPropertyFloat(PROP_CAM_OFFSET_Z, camOffsetZ);
		camOffsetX = GetPropertyFloat(PROP_CAM_OFFSET_X, camOffsetX);
		lastRotationPoint=(Mat_<double>(3, 1) << 0,0,0);
		actRotationPoint=(Mat_<double>(3, 1) << 0,0,0);
		lastThetaRodrigues=0;
		lastSignDataOk=false;

	} else if (stage == StageGraphReady) {

	}
	RETURN_NOERROR;
}

tResult SignMapping::Shutdown(adtf::cFilter::tInitStage stage, ucom::IException **__exception_ptr)
{
	if (stage == StageGraphReady) {

	} else if (stage == StageNormal) {

	} else if (stage == StageFirst) {

	}

	// call base implementation
	return cFilter::Shutdown(stage, __exception_ptr);
}

tResult SignMapping::OnPinEvent(adtf::IPin *source, tInt eventCode, tInt param1, tInt param2, adtf::IMediaSample *mediaSample)
{
	if (eventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		if (mediaSample != NULL && m_pDescriptionInput != NULL)
		{
			if (source == &posePin) {
				READ_MEDIA_PIN(mediaSample, OdometryPose, currentPose);
			}
			else if (source == &signInput) {
				tInt16 i16ID = 0;
				tFloat32 f32Area = 0;
				tFloat32 rvec[3] = {0};
				tFloat32 tvec[3] = {0.0 , 0.0 , 0.0};

				{   // focus for sample read lock
					// read-out the incoming Media Sample
					__adtf_sample_read_lock_mediadescription(m_pDescriptionInput,mediaSample,pCoderInput);

					// get IDs
					if(m_firstTime)
					{
						pCoderInput->GetID("i16Identifier",m_szIDRoadSignI16Identifier);
						pCoderInput->GetID("f32Imagesize", m_szIDRoadSignF32Imagesize);
						pCoderInput->GetID("af32RVec[0]", m_szIDRoadSignExtAf32RVec);
						pCoderInput->GetID("af32TVec[0]", m_szIDRoadSignExtAf32TVec);
						m_firstTime = tFalse;
					}

					// get the values from sample
					pCoderInput->Get(m_szIDRoadSignI16Identifier, (tVoid*)&i16ID);
					pCoderInput->Get(m_szIDRoadSignF32Imagesize, (tVoid*)&f32Area);
					pCoderInput->Get("af32TVec", (tVoid*)tvec),
							pCoderInput->Get("af32RVec", (tVoid*)rvec);
				}
				//if(abs(120 - (rvec[0] * 180/M_PI)) < 20){
				SignStruct output;
				output.id = static_cast<RoadSign>(i16ID);

				Mat origPoint = (Mat_<double>(4, 1) << tvec[0],tvec[1],tvec[2],1);

				Mat rotationPoint = (Mat_<double>(3, 1) << rvec[0],rvec[1],rvec[2]);
				Mat rodriguesResult = (Mat_<double>(3, 3));
				cv::Rodrigues(rotationPoint,rodriguesResult);

				Mat rodriguesResult44 = (Mat_<double>(4, 4) <<	rodriguesResult.at<double>(0,0),	rodriguesResult.at<double>(0,1),	rodriguesResult.at<double>(0,2), 0.0,
																rodriguesResult.at<double>(1,0),	rodriguesResult.at<double>(1,1),	rodriguesResult.at<double>(1,2), 0.0,
																rodriguesResult.at<double>(2,0),	rodriguesResult.at<double>(2,1),	rodriguesResult.at<double>(2,2), 0.0,
																							0,								0,								0,	1);


				Mat origRotationPoint = (Mat_<double>(4, 1) << rvec[0],rvec[1],rvec[2],1);
				Mat diffRotation = (Mat_<double>(3, 1) << 0,0,0);
				if (lastRotationPoint.at<double>(0)==0 && lastRotationPoint.at<double>(1)==0 && lastRotationPoint.at<double>(2)==0){
					;//Startup
					//logger.log()<<"Startup;"<<rotationPoint.at<double>(0)<<";"<<rotationPoint.at<double>(1)<<";"<<rotationPoint.at<double>(2)<<endl;
				}
				else
				{
					diffRotation=rotationPoint-lastRotationPoint;
				}

				lastRotationPoint=rotationPoint;
				actRotationPoint+=diffRotation;
				rotationPoint*=cStdMath::MATH_RAD2DEG;
				//logger.log()<<"rvec;deg;"<<rotationPoint.at<double>(0)<<";"<<rotationPoint.at<double>(1)<<";"<<rotationPoint.at<double>(2)<<endl;
				//LOG_INFO(cString::Format("Original point(camera) x:%g y:%g z:%g",origPoint.at<double>(0),origPoint.at<double>(1),origPoint.at<double>(2)));

				OdometryPose poseRad=currentPose;
				poseRad.theta*=-cStdMath::MATH_DEG2RAD;

				Mat car2World	=	(Mat_<double>(4, 4) <<	cos(poseRad.theta),		sin(poseRad.theta),				0,							poseRad.x,
															-sin(poseRad.theta),	cos(poseRad.theta),				0,							poseRad.y,
															0,						0,								1,							0,
															0,						0,								0,							1);

				//logger.log()<<"Car2World:"<<endl<<car2World<<endl;
				double camAngleRad=camAngle*cStdMath::MATH_DEG2RAD;

				//ROTATION ABOUT LOCAL CAMERA X AXIS
				Mat cam2Car1  	=	(Mat_<double>(4, 4) <<	1,		0,					0,					0,
															0,		cos(camAngleRad),	sin(camAngleRad),	0,
															0,		-sin(camAngleRad),	cos(camAngleRad),	0,
															0,		0,					0,					1);

				//ROTATION ABOUT LOCAL X AXIS 90 DEGRESS
				Mat cam2Car2  	=	(Mat_<double>(4, 4) <<	1,		0,		0,		0,
															0,		0,		1,		0,
															0,		-1,		0,		0,
															0,		0,		0,		1);

				//ROTATION ABOUT LOCAL Z AXIS 90 DEGREES
				Mat cam2Car3  	=	(Mat_<double>(4, 4) <<	0,		1,		0,		0,
															-1,		0,		0,		0,
															0,		0,		1,		0,
															0,		0,		0,		1);

				//MOVE THE LOCAL COORDINATES TO GLOBAL CAR COORDINATES
				Mat cam2Car4  	=	(Mat_<double>(4, 4) <<	1,		0,		0,		camOffsetX,
															0,		1,		0,		camOffsetY,
															0,		0,		1,		camOffsetZ,
															0,		0,		0,		1);

				Mat cam2Car = cam2Car4*cam2Car3*cam2Car2*cam2Car1;
				//logger.log()<<"Cam2Car:"<<endl<<cam2Car<<endl;


				Mat finalPoint = (Mat_<double>(4, 1));
				Mat finalPointRotation = (Mat_<double>(4, 1));
				finalPoint=car2World*(cam2Car*origPoint);

				finalPointRotation=cam2Car3*cam2Car2*cam2Car1*origRotationPoint;
				finalPointRotation*=cStdMath::MATH_RAD2DEG;


				Mat rodriguesMod (Mat_<double>(4, 4));
				rodriguesMod = cam2Car3*cam2Car2*rodriguesResult44;
				double distCartoTarget = sqrt(pow(finalPoint.at<double>(0)-currentPose.x,2)+pow(finalPoint.at<double>(1)-currentPose.y,2));
				double thetaCartoTarget = atan2(finalPoint.at<double>(1)-currentPose.y,fabs(finalPoint.at<double>(0)-currentPose.x));
				double theta;
				double thetaRodrigues;
				double deltaThetaRodrigues;

				thetaRodrigues = cStdMath::MATH_RAD2DEG*atan2(rodriguesResult44.at<double>(2,1),fabs(rodriguesResult44.at<double>(2,2)));
				deltaThetaRodrigues=thetaRodrigues-lastThetaRodrigues;
				lastThetaRodrigues=thetaRodrigues;
				bool signDataOk;
				if (distCartoTarget<2.0 && fabs(thetaRodrigues)<60)
				{
					signDataOk=true;
					if (!lastSignDataOk && signDataOk)
					{
						deltaThetaRodrigues=0;
						logger.log()<<"RodriguesReset;"<<thetaRodrigues<<endl;
					}
				}
				else if (distCartoTarget>2.0)
				{
					deltaThetaRodrigues = thetaCartoTarget;
					signDataOk=true;
				}
				else
				{
					signDataOk=false;
				}
				lastSignDataOk=signDataOk;
				if (signDataOk)
				{
					if (fabs(deltaThetaRodrigues)<15)
					{
						;
					}
					else
					{
						logger.log()<<"RodriguesDeltaTooBig;"<<deltaThetaRodrigues<<endl;
						signDataOk=false;
					}
				}
				else
				{
					thetaRodrigues=0;
				}

				theta = thetaRodrigues+currentPose.theta;


				//logger.log()<<"Rodrigues org;"<<rodriguesResult44<<endl;
				//logger.log()<<"Rodrigues mod;"<<rodriguesMod<<endl;
				logger.log()<<"Dist;"<<distCartoTarget<<endl;
				if (theta<0)
				{
					theta+=360.0;
				}
				else if (theta>360.0)
				{
					theta-=360.0;
				}
				logger.log()<<"Theta;"<<theta<<"ThetaOdo;"<<currentPose.theta<<"ThetaRodrigues;"<<thetaRodrigues<<endl;

				output.pos = Point3d(finalPoint.at<double>(0), finalPoint.at<double>(1), finalPoint.at<double>(2));
				output.theta = theta;
				//LOG_INFO(cString::Format("World pos x:%g y:%g z:%g",finalPoint.at<double>(0),finalPoint.at<double>(1),finalPoint.at<double>(2)));
				//LOG_INFO(cString::Format("World angle x:%g y:%g z:%g",finalPointRotation.at<double>(0),finalPointRotation.at<double>(1),finalPointRotation.at<double>(2)));

				//logger.log()<<"World theta;"<<output.theta<<endl;
				//bool signAligned=fabs(cStdMath::MATH_RAD2DEG*rvec[2])<45;
				//bool signOnGround=finalPoint.at<double>(2)<0.5;
				if (signDataOk)
				{
					//Send to path planner
					cObjectPtr<IMediaSample> sample;
					if (IS_OK(AllocMediaSample(&sample))) {
						sample->Update(_clock->GetStreamTime(), &output, sizeof(SignStruct), 0);
						signOutput.Transmit(sample);
					}
				}

				//logger.log()<<"RVEC:"<<cStdMath::MATH_RAD2DEG*rvec[0]<<";"<<cStdMath::MATH_RAD2DEG*rvec[1]<<";"<<cStdMath::MATH_RAD2DEG*rvec[2]<<endl;


			}


		}
	}else if (eventCode == IPinEventSink::PE_MediaTypeChanged && source != NULL)
	{
		cObjectPtr<IMediaType> pType;
		source->GetMediaType(&pType);
		if (pType != NULL)
		{
			cObjectPtr<IMediaTypeDescription> pMediaTypeDesc;
			RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDesc));

			m_pDescriptionInput = pMediaTypeDesc;
		}
	}

	RETURN_NOERROR;
}

void SignMapping::ProcessInput()
{

}
