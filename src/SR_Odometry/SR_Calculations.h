#ifndef SR_CALCULATIONS_H
#define SR_CALCULATIONS_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include "QuatTypes.h"
#include <odometry_data.h>
#include <math.h>

class SR_Calculations
{
public:
    SR_Calculations();
	tFloat32 CalculateSpeed(const tUInt32 &ui32DeltaTime, const tUInt32 &ui32Ticks);
	tResult CalculateOdometry(tWheelData m_tLastStructRight,tWheelData m_tLastStructLeft,tWheelData m_tBeforeLastStructRight,tWheelData m_tBeforeLastStructLeft);
	tResult calulateEulerAngles(tFloat32 f32Qw, tFloat32 f32Qx, tFloat32 f32Qy, tFloat32 f32Qz, tFloat32 f32A_x, tFloat32 f32A_y, tFloat32 f32A_z);
	void CalculateSteering();
	OdometryPose getData();
	tFloat32 getSpeed();
	tFloat32 getLastDistance();
	void InitCalculations(tFloat32 wheelC, tInt32 slot_Count);
//	void RadiusSmooth();

private:

	/*! the wheel circumference in meter */
	tFloat32 m_f32wheelCircumference;

	/*! the wheel distance in meter */

	/*! number of lines on encoder */
	tInt32 CW_SLOT_COUNT;

	/*! holds the overall X position since starting the adtf config */
	tFloat32 m_f32OverallX;
	tFloat32 m_f32OverallY;
	tFloat32 m_f32PreviousX;
	tFloat32 m_f32PreviousY;
	tFloat32 m_f32PreviousYaw;

	tFloat32 f32Yaw_out;
	tFloat32 f32Yaw_out_0;
	tFloat32 f32Yaw;
	tFloat32 f32Speed;
	tFloat f32distance;

	tBool bFirstYaw;
};

#endif // SR_CALCULATIONS_H
