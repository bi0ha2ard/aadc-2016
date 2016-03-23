#include "SR_Calculations.h"
#include "QuatTypes.h"
#include "EulerAngles.hpp"

#define PI 3.14159265359f

#define CW_ERROR_DIFFERENCE_SIDES 0.40f
//#define CW_ERROR_DIFFERENCE_SIDES 0.30f
#define CW_MIN_LIMIT_IGNORE 0.01f


SR_Calculations::SR_Calculations()
{

}

void SR_Calculations::InitCalculations(tFloat32 wheelC, tInt32 slot_Count)
{
	m_f32wheelCircumference = wheelC;
	CW_SLOT_COUNT = slot_Count;

	m_f32OverallX = 0.0f;
	m_f32OverallY = 0.0f;
	m_f32PreviousX = 0.0f;
	m_f32PreviousY = 0.0f;

	f32Yaw_out = 0;
	f32Yaw_out_0 = 0;
	f32Yaw = 0;
	m_f32PreviousYaw = 0;
	f32Speed = 0;
	f32distance = 0;

	bFirstYaw = tTrue;
}

tFloat32 SR_Calculations::CalculateSpeed(const tUInt32 &ui32DeltaTime, const tUInt32 &ui32Ticks)
{
	//          circumference      SlotsInTimeDiff
	// speed =  -------------- *  -------------
	//           TotalSlots*          TimeDiff

    tFloat32 speed = ((m_f32wheelCircumference/CW_SLOT_COUNT)*static_cast<tFloat32>(ui32Ticks))/(static_cast<tFloat32>(ui32DeltaTime)/static_cast<tFloat32>(1e6));

	return speed;
}

tResult SR_Calculations::CalculateOdometry(tWheelData m_tLastStructRight,tWheelData m_tLastStructLeft,tWheelData m_tBeforeLastStructRight,tWheelData m_tBeforeLastStructLeft)
{
	// static variable for warning outputs to console
	static tInt32 i32WarningCounter = 0;

	// calculate the average of both wheel speeds
	f32Speed = (m_tLastStructRight.f32Speed+m_tLastStructLeft.f32Speed)/2;

	if (fabs((m_tLastStructRight.f32Speed-m_tLastStructLeft.f32Speed))>fabs(m_tLastStructRight.f32Speed)*CW_ERROR_DIFFERENCE_SIDES)
	{
		if (m_tLastStructRight.f32Speed<CW_MIN_LIMIT_IGNORE)
		{
			f32Speed = m_tLastStructLeft.f32Speed;
			if (m_tLastStructLeft.i8WheelDir == 1)
			{
				f32Speed = f32Speed*(-1);
			}
		}
		else if (m_tLastStructLeft.f32Speed<CW_MIN_LIMIT_IGNORE)
		{
			f32Speed = m_tLastStructRight.f32Speed;
			if (m_tLastStructRight.i8WheelDir == 1)
			{
				f32Speed = f32Speed*(-1);
			}
		}
		i32WarningCounter++;
		if (i32WarningCounter%200==0)
		{
			LOG_WARNING(cString::Format("Wheel speed from left and right side are very different. Please check cables and connections! Right: %f, Left: %f, Result: %f",
										m_tLastStructRight.f32Speed,m_tLastStructLeft.f32Speed,f32Speed));
		}

	}
	else
	{
		// if direction is backwards speed should be negative
		if (m_tLastStructLeft.i8WheelDir == 1 && m_tLastStructRight.i8WheelDir == 1)
		{
			f32Speed = f32Speed*(-1);
		}
	}

	// calculate the overall distance
	if (m_tLastStructLeft.ui32ArduinoTimestamp!=0 && m_tLastStructRight.ui32ArduinoTimestamp!=0 && m_tBeforeLastStructLeft.ui32ArduinoTimestamp!=0 && m_tBeforeLastStructRight.ui32ArduinoTimestamp!=0)
	{

		tFloat32 DeltaTRight;
		tFloat32 DeltaTLeft;
		tFloat32 DeltaT;

		DeltaTRight = (m_tLastStructRight.ui32ArduinoTimestamp - m_tBeforeLastStructRight.ui32ArduinoTimestamp)/(static_cast<tFloat32>(1e6));
		DeltaTLeft  = (m_tLastStructLeft.ui32ArduinoTimestamp - m_tBeforeLastStructLeft.ui32ArduinoTimestamp)/(static_cast<tFloat32>(1e6));
		DeltaT = ((DeltaTLeft+DeltaTRight)/(2));
		f32distance = DeltaT*f32Speed;

		if (DeltaT<1.0f && DeltaT>0)
		{
			m_f32OverallX = m_f32PreviousX+f32distance*cos(static_cast<double>(cStdMath::MATH_DEG2RAD*(m_f32PreviousYaw)));
			m_f32OverallY = m_f32PreviousY+f32distance*sin(static_cast<double>(cStdMath::MATH_DEG2RAD*(m_f32PreviousYaw)));

//			if (m_tLastStructRight.f32Speed!=m_tLastStructLeft.f32Speed)
//			{
//				RadiusSmooth();
//			}
			m_f32PreviousYaw = f32Yaw;
			m_f32PreviousX = m_f32OverallX;
			m_f32PreviousY = m_f32OverallY;
		}

	}
	RETURN_NOERROR;
}


// Not precise, not yet used
/*void SR_Calculations::RadiusSmooth()
{

	tFloat32 m_f32Radius;
	tFloat32 rho;

	if (icounter<9)
	{
		icounter++;
	}
	else if (icounter==9)
	{
		rho = 0.5*sqrt(static_cast<double>(pow((m_f32OverallX-f32X0),2)+pow((m_f32OverallY-f32Y0),2)));
		m_f32Radius = rho/(sin(0.5*static_cast<double>(cStdMath::MATH_DEG2RAD*(f32Yaw-f32Yaw0))));

		f32Yaw0 = f32Yaw;
		f32X0 = m_f32OverallX;
		f32Y0 = m_f32OverallY;
		icounter = 0;
	}
}*/

tResult SR_Calculations::calulateEulerAngles(tFloat32 f32Qw, tFloat32 f32Qx, tFloat32 f32Qy, tFloat32 f32Qz,tFloat32 f32A_x,tFloat32 f32A_y,tFloat32 f32A_z)
{
	/*! total of 24 different euler systems. We support only one, yet. */
	/*! For details please see "Graphic Germs IV", Ken Shoemake, 1993 */

	tFloat32 Yaw_out_prev = f32Yaw;
	tFloat32 DeltaYaw;

	Quat q;
	q.w = f32Qw;
	q.x = f32Qx;
	q.y = f32Qy;
	q.z = f32Qz;

	EulerAngles e = Eul_FromQuat(q,EulOrdXYZs);

	f32Yaw_out = static_cast<tFloat32>(cStdMath::MATH_RAD2DEG *e.z);

	if (bFirstYaw==tTrue)
	{
		f32Yaw_out_0 = f32Yaw_out;
		bFirstYaw = tFalse;

	}
	f32Yaw = f32Yaw_out-f32Yaw_out_0;

	DeltaYaw = f32Yaw-Yaw_out_prev;

	if ((DeltaYaw>(180)))
	{
		f32Yaw = f32Yaw_out-f32Yaw_out_0-360;
	}
	else if ((DeltaYaw<(-180)))
	{
		f32Yaw = f32Yaw_out-f32Yaw_out_0+360;
	}

	while (f32Yaw<(0))
	{
		f32Yaw += 360;
	}
	while (f32Yaw>360)
	{
		f32Yaw -=360;
	}

	RETURN_NOERROR;
}

OdometryPose SR_Calculations::getData()
{
	OdometryPose CurrentPose;
	CurrentPose.theta = static_cast<double>(f32Yaw);
	CurrentPose.x = static_cast<double>(m_f32OverallX);
	CurrentPose.y = static_cast<double>(m_f32OverallY);
	CurrentPose.speed = static_cast<double>(f32Speed);

	return CurrentPose;
}
