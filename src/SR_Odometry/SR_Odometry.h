#ifndef _ODOMETRY_H
#define _ODOMETRY_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include "SR_Calculations.h"

#define OID_SR_ODOMETRY "sr.sensor.odometry"

class SR_Odometry : public adtf::cFilter
{
	ADTF_FILTER(OID_SR_ODOMETRY, "SpaceRacer Odometry", adtf::OBJCAT_DataFilter)

public:
	SR_Odometry(const tChar *info);
	virtual ~SR_Odometry();

protected:

    /*! Input pins */
    adtf::cInputPin m_oInputWheelLeft;
    adtf::cInputPin m_oInputWheelRight;
	adtf::cInputPin m_oInputInerMeasUnit;

    /*! Output pins */
	adtf::cOutputPin m_oOutputPose;

    tResult Init(tInitStage eStage, __exception);

    tResult OnPinEvent(adtf::IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, adtf::IMediaSample *pMediaSample);

private:
	int myOdoTransmit;
	tResult TransmitPose(OdometryPose p);

    /*! Descriptions for input/output pins */

    /*! descriptor for wheel sensor data */
    cObjectPtr<adtf::IMediaTypeDescription> m_pDescriptionWheelDataLeft;
    /*! the id for the ui32WheelTach of the media description for input pin of the WheelData data */
    adtf::tBufferID m_szIDWheelDataLeftUi32WheelTach;
    /*! the id for the i8WheelDir of the media description for input pin of the WheelData data */
    adtf::tBufferID m_szIDWheelDataLeftI8WheelDir;
    /*! the id for the arduino time stamp of the media description for input pin of the WheelData data */
    adtf::tBufferID m_szIDWheelDataLeftArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsWheelDataLeftSet;

    /*! descriptor for wheel sensor data */
    cObjectPtr<adtf::IMediaTypeDescription> m_pDescriptionWheelDataRight;
    /*! the id for the ui32WheelTach of the media description for input pin of the WheelData data */
    adtf::tBufferID m_szIDWheelDataRightUi32WheelTach;
    /*! the id for the i8WheelDir of the media description for input pin of the WheelData data */
    adtf::tBufferID m_szIDWheelDataRightI8WheelDir;
    /*! the id for the arduino time stamp of the media description for input pin of the WheelData data */
    adtf::tBufferID m_szIDWheelDataRightArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsWheelDataRightSet;

    /*! descriptor for inertcial measurement unit sensor data */
	cObjectPtr<adtf::IMediaTypeDescription> m_pDescriptionInerMeasUnitData;
    /*! the id for the f32Q_w of the media description for output pin of the imu data */
	adtf::tBufferID m_szIDInerMeasUnitF32Q_w;
    /*! the id for the f32Q_x of the media description for output pin of the imu data */
	adtf::tBufferID m_szIDInerMeasUnitF32Q_x;
    /*! the id for the f32Q_y of the media description for output pin of the imu data */
	adtf::tBufferID m_szIDInerMeasUnitF32Q_y;
    /*! the id for the f32Q_z of the media description for output pin of the imu data */
	adtf::tBufferID m_szIDInerMeasUnitF32Q_z;
    /*! the id for the f32A_x of the media description for output pin of the imu data */
	adtf::tBufferID m_szIDInerMeasUnitF32A_x;
    /*! the id for the f32A_y of the media description for output pin of the imu data */
	adtf::tBufferID m_szIDInerMeasUnitF32A_y;
    /*! the id for the f32A_z of the media description for output pin of the imu data */
	adtf::tBufferID m_szIDInerMeasUnitF32A_z;
    /*! the id for the arduino time stamp of the media description for output pin of the imu data */
	adtf::tBufferID m_szIDInerMeasUnitArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsInerMeasUnitSet;

    /*! first sample was received from left wheel */
    tBool m_bSampleReceivedLeftWheel;
    tBool m_bSampleReceivedRightWheel;
	tBool m_bPosCalculated;

	tInt32 rightcounter;
	tInt32 leftcounter;
	tInt32 imucounter;

    /*! the timestamp of the last left wheel struct */
    tWheelData m_tLastStructRight;
    tWheelData m_tBeforeLastStructRight;
    tWheelData m_tLastStructLeft;
    tWheelData m_tBeforeLastStructLeft;

	SR_Calculations calc;

};


#endif
