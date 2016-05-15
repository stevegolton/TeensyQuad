#ifndef FLIGHT_H
#define FLIGHT_H

#include <stdint.h>
#include <stddef.h>
#include "vector3f.h"

#define NUM_MOTORS			( 4 )
#define NUM_RCVR_CHANNELS	( 6 )

typedef void (*set_rotor_spd_t)( const size_t rotor_number, const uint16_t spd );

// Describes a receiver input configuration
// Roll, elev and yaw range from -1 to +1 and throttle, varA and varB range from
// 0 to 1.
typedef struct
{
	float fRoll;
	float fPitch;
	float fThrottle;
	float fYaw;
	float fVarA;
	float fVarB;

} stReceiverInput_t;

typedef struct
{
	float fFL;
	float fFR;
	float fRL;
	float fRR;

} stMotorDemands_t;

/**
 * @brief		Initialise the flight controller.
 */
void flight_setup( void );

/**
 * @brief		Updates the flight controller with a given set of IMU and
 * 				receiver values.
 * @param[in]	uiTimestep		Time in milliseconds since the last time we were called.
 * @param[in]	stAccel			Current accelerometer readings in g.
 * @param[in]	stGyro			Current gyroscope readings in rad/sec.
 * @param[in]	stReceiverInput	Current receiver input values.
 * @param[out]	pstMotorDemands	Pointer to where to put the resulting receiver values.
 */
void flight_process( uint16_t uiTimestep,
					 vector3f_t *pstAccel,
					 vector3f_t *pstGyro,
					 vector3f_t *pstMag,
					 stReceiverInput_t *pstReceiverInput,
					 stMotorDemands_t *pstMotorDemands );

/**
 * @brief		Sets the trim.
 * @param[in]	pstTrim		Pointer to the new trim.
 */
void FLIGHT_SetTrim( const vector3f_t *const pstTrim );
void FLIGHT_SetPidGains( const float fRateP, const float fRateD, const float fAngleP );
void FLIGHT_GetRotation( vector3f_t *pstRotation );

#endif
