#include <stdint.h>
#include <stddef.h>

#include "flight.h"
#include "common.h"
#include "vector3f.h"
#include "sensor_fusion.h"
#include "pid.h"

static set_rotor_spd_t _set_rotor_spd;
static get_recvr_channel_t _get_recvr_channel;
static uint16_t _receiver_max;
static uint16_t _receiver_center;
static stSENSORFUSION_Cxt_t stSensorFusion;

static stPidCxt_t stPIDElevRate;
static stPidCxt_t stPIDElevAngle;

static stPidCxt_t stPIDRollRate;
static stPidCxt_t stPIDRollAngle;

static stPidCxt_t stPIDYawRate;

static float rxThrottle;
static float rxPitch;
static float rxRoll;
static float rxYaw;
static float rxVra;
static float rxVrb;

static vector3f_t trim;

#define DEFGAIN_RATE_P (0.48)
#define DEFGAIN_RATE_I (0)
#define DEFGAIN_RATE_D (0.022)

#define DEFGAIN_ANGLE_P (0.92)
#define DEFGAIN_ANGLE_I (0)
#define DEFGAIN_ANGLE_D (0)

static uint32_t ScaleDecimalToTicks( float decimal );

void flight_setup( set_rotor_spd_t set_rotor_spd,
				   get_recvr_channel_t get_recvr_channel,
				   int16_t receiver_max )
{
	// Cache local copies of callback functions
	_set_rotor_spd = set_rotor_spd;
	_get_recvr_channel = get_recvr_channel;
	_receiver_max = receiver_max;
	_receiver_center = ( receiver_max / 2 );

	SENSORFUSION_Setup( &stSensorFusion );


	PID_Setup( &stPIDElevAngle, DEFGAIN_ANGLE_I, DEFGAIN_ANGLE_P, DEFGAIN_ANGLE_D, 0, 0 );
	PID_Setup( &stPIDElevRate, DEFGAIN_RATE_I, DEFGAIN_RATE_P, DEFGAIN_RATE_D, 0, 0 );

	PID_Setup( &stPIDRollAngle, DEFGAIN_ANGLE_I, DEFGAIN_ANGLE_P, DEFGAIN_ANGLE_D, 0, 0 );
	PID_Setup( &stPIDRollRate, DEFGAIN_RATE_I, DEFGAIN_RATE_P, DEFGAIN_RATE_D, 0, 0 );

	PID_Setup( &stPIDYawRate, DEFGAIN_RATE_I, DEFGAIN_RATE_P, DEFGAIN_RATE_D, 0, 0 );

	return;
}

/*
 * Motor Orientation (as viewed from above)
 *
 *         ^
 *         |
 *       front
 *
 * M4-ACW       M3-CW
 *    \          /
 *     \        /
 *      \      /
 *       -----
 *      /      \
 *     /        \
 *    /          \
 * M1-CW        M2-ACW
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

void flight_process( uint16_t timestep_ms, vector3f_t accel, vector3f_t gyro )
{
	int rcvr_idx;
	uint16_t rcvr_values[NUM_RCVR_CHANNELS];
	vector3f_t *pstRotation;
	float fElevRate;
	float fElevAngleErr = 0;
	float fElevThrottle;

	float fRollRate;
	float fRollAngleErr = 0;
	float fRollThrottle;

	float fYawRateErr = 0;

	float fYawThrottle;

	float fTimeStep = (float)timestep_ms / 1000;

	// Fetch receiver inputs
	for ( rcvr_idx = 0; rcvr_idx < NUM_RCVR_CHANNELS; rcvr_idx++ )
	{
		rcvr_values[rcvr_idx] = _get_recvr_channel( rcvr_idx );
	}

	// Update sensor fusion
	pstRotation = SENSORFUSION_Update( &stSensorFusion,
									   &gyro,
									   &accel,
									   fTimeStep );

	/* Work out input values as floats */
	rxRoll = ( (float)( (int32_t)rcvr_values[0] - _receiver_center ) ) / _receiver_max;
	rxPitch = ( (float)( (int32_t)rcvr_values[1] - _receiver_center ) ) / _receiver_max;
	rxThrottle = ( (float)rcvr_values[2] ) / _receiver_max;
	rxYaw = ( (float)( (int32_t)rcvr_values[3] - _receiver_center ) ) / _receiver_max;
	rxVra = ( (float)rcvr_values[4] ) / _receiver_max;
	rxVrb = ( (float)rcvr_values[5] ) / _receiver_max;

	// If throttle is small.. don't fly
	if ( rxThrottle < 0.1 )
	{
		_set_rotor_spd( 0, 0 );
		_set_rotor_spd( 1, 0 );
		_set_rotor_spd( 2, 0 );
		_set_rotor_spd( 3, 0 );
	}
	else
	{
		// We are flying... calculate PIDS
		fElevAngleErr = ( ( (pstRotation->y) / 90 ) + trim.y ) - ( rxPitch * rxVra );
		fRollAngleErr = ( ( (pstRotation->x) / 90 ) + trim.x ) - ( rxRoll * rxVra );

		fElevRate = PID_Update( &stPIDElevAngle, fElevAngleErr, fTimeStep );
		fElevThrottle = PID_Update( &stPIDElevRate, fElevRate + (gyro.y/500), fTimeStep );

		fRollRate = PID_Update( &stPIDRollAngle, fRollAngleErr, fTimeStep );
		fRollThrottle = PID_Update( &stPIDRollRate, fRollRate + (gyro.x/500), fTimeStep );

		fYawRateErr = trim.z + ( rxYaw * rxVrb );
		fYawThrottle = PID_Update( &stPIDYawRate, fYawRateErr + (gyro.z/500), fTimeStep );

		_set_rotor_spd( 0, ScaleDecimalToTicks( rxThrottle + fElevThrottle + fRollThrottle - fYawThrottle ) ); // M3
		_set_rotor_spd( 1, ScaleDecimalToTicks( rxThrottle + fElevThrottle - fRollThrottle + fYawThrottle ) ); // M4
		_set_rotor_spd( 2, ScaleDecimalToTicks( rxThrottle - fElevThrottle - fRollThrottle - fYawThrottle ) ); // M1
		_set_rotor_spd( 3, ScaleDecimalToTicks( rxThrottle - fElevThrottle + fRollThrottle + fYawThrottle ) ); // M2
	}

	return;
}

static uint32_t ScaleDecimalToTicks( float decimal )
{
	return (uint32_t)( decimal * _receiver_max );
}
