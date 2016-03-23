#include "SR_Odometry.h"
#include "../misc/PinWrapper.h"
#include <odometry_data.h>

using namespace adtf;

#define PI 3.14159265359f

// Create filter shell
ADTF_FILTER_PLUGIN("SpaceRacer Odometry", OID_SR_ODOMETRY, SR_Odometry)

SR_Odometry::SR_Odometry(const tChar *info) : cFilter(info)
{
	SetPropertyFloat("Longitudinal wheel distance",0.365);
	SetPropertyFloat("Wheel circumference",0.34);
	SetPropertyFloat("Wheel distance",0.26);
	SetPropertyInt("Slot Count",60);
	SetPropertyFloat("Wheel circumference" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Wheel circumference" NSSUBPROP_DESCRIPTION, "Set the wheel circumference in meter here");
	SetPropertyStr("Wheel distance" NSSUBPROP_DESCRIPTION,"Seth the wheel distance in meter here");
}

SR_Odometry::~SR_Odometry()
{

}

tResult SR_Odometry::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

	if (eStage == StageFirst)
	{
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

		//get description for inertial measurement sensor data pin
		tChar const * strDescInerMeasUnit = pDescManager->GetMediaDescription("tInerMeasUnitData");
		RETURN_IF_POINTER_NULL(strDescInerMeasUnit);
		cObjectPtr<IMediaType> pTypeInerMeasUnit = new cMediaType(0, 0, 0, "tInerMeasUnitData", strDescInerMeasUnit, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		//get description for wheel sensors data pins
		tChar const * strDescWheelData = pDescManager->GetMediaDescription("tWheelData");
		RETURN_IF_POINTER_NULL(strDescWheelData);
		cObjectPtr<IMediaType> pTypeWheelData = new cMediaType(0, 0, 0, "tWheelData", strDescWheelData, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		//create pin for wheel left sensor data
		RETURN_IF_FAILED(m_oInputWheelLeft.Create("WheelLeft_Struct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputWheelLeft));

		//create pin for wheel right data
		RETURN_IF_FAILED(m_oInputWheelRight.Create("WheelRight_Struct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputWheelRight));

		//create pin for inertial measurement unit data
		RETURN_IF_FAILED(m_oInputInerMeasUnit.Create("InerMeasUnit_Struct", pTypeInerMeasUnit, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputInerMeasUnit));

		//get mediatype description for wheel sensor data type
		RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelDataRight));

		RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelDataLeft));

		RETURN_IF_FAILED(pTypeInerMeasUnit->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInerMeasUnitData));

		// Pose output pin
		cObjectPtr<IMediaType> poseType;
		RETURN_IF_FAILED(AllocMediaType(&poseType, MEDIA_TYPE_ODOMETRY, MEDIA_SUBTYPE_ODOMETRY_POSE))
		RETURN_IF_FAILED(m_oOutputPose.Create("Pose", poseType, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputPose));

	}
	else if ((eStage == StageNormal) || (eStage == StageGraphReady))
	{
		tFloat32 wheelC = static_cast<tFloat32>(GetPropertyFloat("wheel circumference"));
		tInt32 slot_Count = static_cast<tInt32>(GetPropertyFloat("Slot count"));
		calc.InitCalculations(wheelC,slot_Count);

		rightcounter = 0;
		leftcounter = 0;
		imucounter = 0;

		m_bSampleReceivedLeftWheel = tFalse;
		m_bSampleReceivedRightWheel = tFalse;
		m_bPosCalculated = tFalse;

		m_bIDsWheelDataLeftSet = tFalse;
		m_bIDsWheelDataRightSet = tFalse;
		m_bIDsInerMeasUnitSet = tFalse;

		m_tBeforeLastStructLeft.ui32ArduinoTimestamp = 0;
		m_tBeforeLastStructRight.ui32ArduinoTimestamp = 0;

		myOdoTransmit=1000;
/*
		if (!m_oInputWheelLeft.IsConnected() || !m_oInputWheelRight.IsConnected() || !m_oInputInerMeasUnit.IsConnected())
		{
			LOG_ERROR("Odometry: At least one input pin is not connected: please check connections!");
			RETURN_ERROR(ERR_NOT_CONNECTED);
		}
*/
	}

	RETURN_NOERROR;
}

tResult SR_Odometry::OnPinEvent(adtf::IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, adtf::IMediaSample* pMediaSample)
{

	myOdoTransmit--;
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL)
	{
		if (pSource == &m_oInputWheelLeft)
		{

			tUInt32 ui32Tach = 0;
			tInt8 i8Direction = 0;
			tUInt32 ui32Timestamp = 0;
			tInt32 Tach = 0;

			{
				__adtf_sample_read_lock_mediadescription(m_pDescriptionWheelDataLeft,pMediaSample,pCoderInput);

				//get IDs
				if(!m_bIDsWheelDataLeftSet)
				{
					pCoderInput->GetID("ui32WheelTach",m_szIDWheelDataLeftUi32WheelTach);
					pCoderInput->GetID("i8WheelDir",m_szIDWheelDataLeftI8WheelDir);
					pCoderInput->GetID("ui32ArduinoTimestamp",m_szIDWheelDataLeftArduinoTimestamp);
					m_bIDsWheelDataLeftSet = tTrue;
				}

				pCoderInput->Get(m_szIDWheelDataLeftArduinoTimestamp,(tVoid*)&ui32Timestamp);
				pCoderInput->Get(m_szIDWheelDataLeftI8WheelDir,(tVoid*)&i8Direction);
				pCoderInput->Get(m_szIDWheelDataLeftUi32WheelTach,(tVoid*)&ui32Tach);

				m_tLastStructLeft.i8WheelDir = i8Direction;
				m_tLastStructLeft.ui32ArduinoTimestamp = ui32Timestamp;
				m_tLastStructLeft.ui32WheelTach = ui32Tach;
				m_tLastStructLeft.f32Speed = 0;
			}


			if (m_tBeforeLastStructLeft.ui32ArduinoTimestamp > 0) {
				if (m_bSampleReceivedLeftWheel==tFalse)
				{
					Tach = ui32Tach-m_tBeforeLastStructLeft.ui32WheelTach;
					leftcounter++;
					if (Tach>0 && ui32Timestamp != m_tBeforeLastStructLeft.ui32ArduinoTimestamp)
					{
						m_tLastStructLeft.f32Speed = calc.CalculateSpeed(ui32Timestamp-m_tBeforeLastStructLeft.ui32ArduinoTimestamp,Tach);
						m_bSampleReceivedLeftWheel = tTrue;
					}
					else
					{
						if (ui32Timestamp == m_tBeforeLastStructLeft.ui32ArduinoTimestamp) {
						}
						m_tLastStructLeft.f32Speed =0.0;
					}
				}
			} else {
				m_tBeforeLastStructLeft = m_tLastStructLeft;
			}

		}
		else if (pSource == &m_oInputWheelRight)
		{
			tUInt32 ui32Tach = 0;
			tInt8 i8Direction = 0;
			tUInt32 ui32Timestamp = 0;
			tInt32 Tach = 0;

			{
				__adtf_sample_read_lock_mediadescription(m_pDescriptionWheelDataRight,pMediaSample,pCoderInput);

				//get IDs
				if(!m_bIDsWheelDataRightSet)
				{
					pCoderInput->GetID("i8WheelDir",m_szIDWheelDataRightI8WheelDir);
					pCoderInput->GetID("ui32WheelTach",m_szIDWheelDataRightUi32WheelTach);
					pCoderInput->GetID("ui32ArduinoTimestamp",m_szIDWheelDataRightArduinoTimestamp);
					m_bIDsWheelDataRightSet = tTrue;
				}

				pCoderInput->Get(m_szIDWheelDataRightArduinoTimestamp,(tVoid*)&ui32Timestamp);
				pCoderInput->Get(m_szIDWheelDataRightI8WheelDir,(tVoid*)&i8Direction);
				pCoderInput->Get(m_szIDWheelDataRightUi32WheelTach,(tVoid*)&ui32Tach);

				m_tLastStructRight.i8WheelDir = i8Direction;
				m_tLastStructRight.ui32ArduinoTimestamp = ui32Timestamp;
				m_tLastStructRight.ui32WheelTach = ui32Tach;
				m_tLastStructRight.f32Speed = 0;
			}


			if (m_tBeforeLastStructRight.ui32ArduinoTimestamp > 0) {
				if (m_bSampleReceivedRightWheel==tFalse)
				{
					Tach = ui32Tach-m_tBeforeLastStructRight.ui32WheelTach;
					rightcounter++;
					if (Tach>0 && ui32Timestamp != m_tBeforeLastStructRight.ui32ArduinoTimestamp)
					{
						m_tLastStructRight.f32Speed = calc.CalculateSpeed(ui32Timestamp-m_tBeforeLastStructRight.ui32ArduinoTimestamp,Tach);
						m_bSampleReceivedRightWheel = tTrue;
					}
					else
					{
						if (ui32Timestamp == m_tBeforeLastStructRight.ui32ArduinoTimestamp) {
						}
						m_tLastStructRight.f32Speed =0.0;
					}
				}
			} else {
				m_tBeforeLastStructRight = m_tLastStructRight;
			}

		}
		else if (pSource == &m_oInputInerMeasUnit)
		{
			//write values with zero
			tFloat32 f32Q_w = 0.0f;
			tFloat32 f32Q_x = 0.0f;
			tFloat32 f32Q_y = 0.0f;
			tFloat32 f32Q_z = 0.0f;
			tFloat32 f32A_x = 0.0f;
			tFloat32 f32A_y = 0.0f;
			tFloat32 f32A_z = 0.0f;
			tUInt32 ui32ArduinoTimestamp = 0;

			{   // focus for sample read lock
				// read-out the incoming Media Sample
				__adtf_sample_read_lock_mediadescription(m_pDescriptionInerMeasUnitData,pMediaSample,pCoderInput);

				// get the IDs for the items in the media sample
				if(!m_bIDsInerMeasUnitSet)
				{
					pCoderInput->GetID("f32Q_w", m_szIDInerMeasUnitF32Q_w);
					pCoderInput->GetID("f32Q_x", m_szIDInerMeasUnitF32Q_x);
					pCoderInput->GetID("f32Q_y", m_szIDInerMeasUnitF32Q_y);
					pCoderInput->GetID("f32Q_z", m_szIDInerMeasUnitF32Q_z);
					pCoderInput->GetID("f32A_x", m_szIDInerMeasUnitF32A_x);
					pCoderInput->GetID("f32A_y", m_szIDInerMeasUnitF32A_y);
					pCoderInput->GetID("f32A_z", m_szIDInerMeasUnitF32A_z);
					pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDInerMeasUnitArduinoTimestamp);
					m_bIDsInerMeasUnitSet = tTrue;
				}

				//write date to the media sample with the coder of the descriptor
				pCoderInput->Get(m_szIDInerMeasUnitF32Q_w, (tVoid*)&f32Q_w);
				pCoderInput->Get(m_szIDInerMeasUnitF32Q_x, (tVoid*)&f32Q_x);
				pCoderInput->Get(m_szIDInerMeasUnitF32Q_y, (tVoid*)&f32Q_y);
				pCoderInput->Get(m_szIDInerMeasUnitF32Q_z, (tVoid*)&f32Q_z);
				pCoderInput->Get(m_szIDInerMeasUnitF32A_x, (tVoid*)&f32A_x);
				pCoderInput->Get(m_szIDInerMeasUnitF32A_y, (tVoid*)&f32A_y);
				pCoderInput->Get(m_szIDInerMeasUnitF32A_z, (tVoid*)&f32A_z);
				pCoderInput->Get(m_szIDInerMeasUnitArduinoTimestamp, (tVoid*)&ui32ArduinoTimestamp);

			}

			if ((m_tLastStructRight.f32Speed!=0) && (m_tLastStructLeft.f32Speed!=0))
			{
				// calculate the euler agnles from quaternions
				calc.calulateEulerAngles(f32Q_w,f32Q_x,f32Q_y,f32Q_z,f32A_x,f32A_y,f32A_z);
			}

			imucounter++;
		}

		if ((rightcounter==leftcounter) && (rightcounter>0) && (leftcounter > 0))
		{

			calc.CalculateOdometry(m_tLastStructRight,m_tLastStructLeft,m_tBeforeLastStructRight,m_tBeforeLastStructLeft);
			TransmitPose(calc.getData());

			m_tBeforeLastStructLeft = m_tLastStructLeft;
			m_tBeforeLastStructRight = m_tLastStructRight;
			m_bSampleReceivedLeftWheel = tFalse;
			m_bSampleReceivedRightWheel = tFalse;
		}
	}
	if (myOdoTransmit<0)
	{
		//No pose has been sent, try to reset consistency variables
		LOG_INFO("RESET Left counter ["+cString::FromInt(leftcounter)+"] Right counter ["+cString::FromInt(rightcounter)+"] IMU counter ["+cString::FromInt(imucounter)+"]." );
		myOdoTransmit=300;
		leftcounter=0;
		rightcounter=0;
	}
	RETURN_NOERROR;
}

tResult SR_Odometry::TransmitPose(OdometryPose p)
{
	myOdoTransmit=300;
	if (p.x != p.x || p.y != p.y || p.theta != p.theta) {
		LOG_INFO("NAN in odometry");
	}
	cObjectPtr<IMediaSample> pNewSample;
	if (IS_OK(AllocMediaSample(&pNewSample))) {
		pNewSample->Update(_clock->GetStreamTime(), &p, sizeof(OdometryPose), 0);
		m_oOutputPose.Transmit(pNewSample);
	}
	RETURN_NOERROR;
}
