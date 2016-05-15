#include <stdint.h>
#include <stddef.h>
#include <stdio.h>			// printf & friends
#include <string.h>			// memset & friends

#include "flight.h"
#include "common.h"
#include "vector3f.h"
#include "sensor_fusion.h"
#include "pid.h"

#define PIDGAIN_RATE_I (0)
#define PIDGAIN_RATE_P (0.0048)
#define PIDGAIN_RATE_D (0.00006)

#define PIDGAIN_ANGLE_P (4.0)
#define PIDGAIN_ANGLE_I (0)
#define PIDGAIN_ANGLE_D (0)

#define THRESHOLD_THROT_FLIGHT		( 0.1f )

// PID structures
static stPidCxt_t stPIDPitchRate;
static stPidCxt_t stPIDPitchAngle;
static stPidCxt_t stPIDRollRate;
static stPidCxt_t stPIDRollAngle;
static stPidCxt_t stPIDYawRate;

static stSENSORFUSION_Cxt_t stSensorFusion;

static vector3f_t stTrim;
static uint32_t _uiTimestamp;
static vector3f_t stRotation;

/* ************************************************************************** */
void flight_setup( void )
{
	// Set up our sensor function module
	SENSORFUSION_Setup( &stSensorFusion );

	// Initialise PIDs
	PID_Setup( &stPIDPitchAngle, PIDGAIN_ANGLE_I, PIDGAIN_ANGLE_P, PIDGAIN_ANGLE_D, 0, 0 );
	PID_Setup( &stPIDPitchRate, PIDGAIN_RATE_I, PIDGAIN_RATE_P, PIDGAIN_RATE_D, 0, 0 );

	PID_Setup( &stPIDRollAngle, PIDGAIN_ANGLE_I, PIDGAIN_ANGLE_P, PIDGAIN_ANGLE_D, 0, 0 );
	PID_Setup( &stPIDRollRate, PIDGAIN_RATE_I, PIDGAIN_RATE_P, PIDGAIN_RATE_D, 0, 0 );

	PID_Setup( &stPIDYawRate, PIDGAIN_RATE_I, PIDGAIN_RATE_P, PIDGAIN_RATE_D, 0, 0 );

	_uiTimestamp = 0;

	return;
}

/*
 * Motor Orientation (as viewed from above)
 *
 *         ^
 *         |
 *       front
 *
 * M2-ACW       M1-CW
 *    \          /
 *     \        /
 *      \      /
 *       -----
 *      /      \
 *     /        \
 *    /          \
 * M3-CW        M4-ACW
 *
 * Sensor Fusion Orientations
 * X +ve roll  = right side down
 * Y +ve pitch = nose down
 * Z +ve yaw   = ACW turn (as viewed from above)
 *
 * RX0 = Roll
 * RX1 = Pitch
 * RX2 = Throttle
 * RX3 = Yaw
 * RX4 = VRA
 * RX5 = VRB
 */
/* ************************************************************************** */
void flight_process( uint16_t uiTimestep,
					 vector3f_t *pstAccel,
					 vector3f_t *pstGyro,
					 stReceiverInput_t *pstReceiverInput,
					 stMotorDemands_t *pstMotorDemands )
{
	float fTimeStep;
	static uint16_t uiDecimation;

	float fAngleErrRoll;
	float fAngleErrPitch;

	float fRateTargetRoll;
	float fRateTargetPitch;

	float fRateErrRoll;
	float fRateErrPitch;
	float fRateErrYaw;

	float fAccelTargetRoll;
	float fAccelTargetPitch;
	float fAccelTargetYaw;

	// Work out the timestep as a float
	fTimeStep = ( (float)uiTimestep / 1000 );
	_uiTimestamp += uiTimestep;

	// Update sensor fusion module
	SENSORFUSION_Update( &stSensorFusion,
						 pstGyro,
						 pstAccel,
						 &stRotation,
						 fTimeStep );

	// Apply trim
	stRotation = VECTOR3F_Subtract( stRotation, stTrim );

	// TODO Remove debug code
#if 0
	if ( 100 <= uiDecimation++ )
	{
		uiDecimation = 0;
		printf( "Angle = %d: %d, %d, %d\r\n",
				(int)_uiTimestamp,
				(int)( ( stRotation.x ) * 1000 ),
				(int)( ( stRotation.y ) * 1000 ),
				(int)( ( stRotation.z ) * 1000 )
				);
	}
#endif

	// If throttle is small.. don't fly
	if ( THRESHOLD_THROT_FLIGHT > pstReceiverInput->fThrottle )
	{
		// Throttle is too small - turn all the motors off
		pstMotorDemands->fFL = 0.0f;
		pstMotorDemands->fFR = 0.0f;
		pstMotorDemands->fRL = 0.0f;
		pstMotorDemands->fRR = 0.0f;
	}
	else
	{
		// Throttle is significant - let's fly!

		// Calculate roll and pitch errors
		// These are the equal to the pitch and roll inputs from the receiver
		// minus the actual pitch and roll inputs from the IMU.
		// The inputs from the roll and elevation stick is multiplied by the VRA
		// input to allow an element of adjustment. The max requested angle
		// is +- 0.5 radians which is around +-30 degrees.
		fAngleErrRoll = ( ( pstReceiverInput->fRoll * pstReceiverInput->fVarA ) - stRotation.x );
		fAngleErrPitch = ( ( pstReceiverInput->fPitch * pstReceiverInput->fVarB ) - stRotation.y );

		// Update the PIDs
		// First we feed our angular error to the "angle" PID which will give us
		// the desired angular speed we need to achieve in order to fix the
		// angular error.
		fRateTargetRoll = PID_Update( &stPIDRollAngle, fAngleErrRoll, fTimeStep );
		fRateTargetPitch = PID_Update( &stPIDPitchAngle, fAngleErrPitch, fTimeStep );

		// Then we find the difference between this desired angular speed and
		// the current angular speed (from the gyroscope) to obtain an error
		// factor in the current angular speed.
		// For example if we are rotating exactly at the desired angular speed,
		// then this error will be 0 and no offset needs to be applied to the
		// motors in this axis.
		fRateErrRoll = ( -fRateTargetRoll + pstGyro->x );
		fRateErrPitch = ( -fRateTargetPitch + pstGyro->y );
		fRateErrYaw = ( pstReceiverInput->fYaw + ( pstGyro->z / 300.0f ) );

		// The result of this PID will give us the desired angular acceleration
		// which we can feed directly to the motors.
		fAccelTargetRoll = PID_Update( &stPIDRollRate, fRateErrRoll, fTimeStep );
		fAccelTargetPitch = PID_Update( &stPIDPitchRate, fRateErrPitch, fTimeStep );
		fAccelTargetYaw = PID_Update( &stPIDYawRate, fRateErrYaw + pstGyro->z, fTimeStep );

		// Set the motor values with offsets applied
		pstMotorDemands->fFL = ( pstReceiverInput->fThrottle + fAccelTargetPitch - fAccelTargetRoll + fAccelTargetYaw );
		pstMotorDemands->fFR = ( pstReceiverInput->fThrottle + fAccelTargetPitch + fAccelTargetRoll - fAccelTargetYaw );
		pstMotorDemands->fRL = ( pstReceiverInput->fThrottle - fAccelTargetPitch - fAccelTargetRoll - fAccelTargetYaw );
		pstMotorDemands->fRR = ( pstReceiverInput->fThrottle - fAccelTargetPitch + fAccelTargetRoll + fAccelTargetYaw );
	}

	return;
}

/* ************************************************************************** */
void FLIGHT_SetTrim( const vector3f_t *const pstTrim )
{
	memcpy( &stTrim, pstTrim, sizeof( vector3f_t ) );

	return;
}

/* ************************************************************************** */
void FLIGHT_GetRotation( vector3f_t *pstRotation )
{
	*pstRotation = stRotation;

	return;
}
