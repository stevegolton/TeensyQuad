#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>		// <- TODO how efficient is this - used for atan2() (see below)?
#include <string.h>

#include "sensor_fusion.h"
#include "kalman.h"
#include "vector3f.h"
#include "MadgwickAHRS.h"

#define PI					( 3.14159265359f )
#define RAD2DEG				( 180/PI )
#define fRatioGyro			( 0.99 )
#define fRatioAccel			( 1 - fRatioGyro )

// Which sensor fusion algorithm to use
#define KALMAN
//#define MADGWICK
//#define COMPLIMENTARY

static float GetMag( float x, float y, float z );

/* ************************************************************************** */
void SENSORFUSION_Setup( stSENSORFUSION_Cxt_t *pstCxt )
{
	// Reset rotation vector
	pstCxt->stRotation.x = 0;
	pstCxt->stRotation.y = 0;
	pstCxt->stRotation.z = 0;

#ifdef KALMAN
	KALMAN_Setup( &pstCxt->stKalmanPitch );
	KALMAN_Setup( &pstCxt->stKalmanRoll );
#endif
}

/* ************************************************************************** */
void SENSORFUSION_Update( stSENSORFUSION_Cxt_t *pstCxt,
						  vector3f_t *pstGyro,
						  vector3f_t *pstAccel,
						  vector3f_t *pstMag,
						  vector3f_t *pstRotation,
						  float fTimestep_s )
{
#ifdef KALMAN
	float pitchRate = pstGyro->y;
	float pitchAngle = atan2f( -pstAccel->x, GetMag( pstAccel->z, pstAccel->y, 0 ) );
	float rollRate = pstGyro->x;
	float rollAngle = atan2f( pstAccel->y, GetMag( pstAccel->z, pstAccel->x, 0 ) );

	pstCxt->stRotation.y = KALMAN_Update( &pstCxt->stKalmanPitch, pitchRate, pitchAngle, fTimestep_s );
	pstCxt->stRotation.x = KALMAN_Update( &pstCxt->stKalmanRoll, rollRate, rollAngle, fTimestep_s );
	pstCxt->stRotation.z = pstGyro->z * fTimestep_s + pstCxt->stRotation.z;

	memcpy( pstRotation, &pstCxt->stRotation, sizeof( vector3f_t ) );
#elif defined COMPLIMENTARY
	// Complimentary filter
	pstCxt->stRotation.x = ( fRatioGyro * ( pstGyro->x * fTimestep_s + pstCxt->stRotation.x ) )
			+ ( fRatioAccel * ( atan2f( pstAccel->y, GetMag( pstAccel->z, pstAccel->x, 0 ) ) ) );
	pstCxt->stRotation.y = ( fRatioGyro * ( pstGyro->y * fTimestep_s + pstCxt->stRotation.y ) )
			- ( fRatioAccel * ( atan2f( pstAccel->x, GetMag( pstAccel->z, pstAccel->y, 0 ) ) ) );

	memcpy( pstRotation, &pstCxt->stRotation, sizeof( vector3f_t ) );
#elif defined MADGWICK

	MadgwickAHRSupdateIMU( pstGyro->x, pstGyro->y, pstGyro->z,
						   pstAccel->x, pstAccel->y, pstAccel->z );

	/*
	MadgwickQuaternionUpdate( pstGyro->x, pstGyro->y, pstGyro->z,
							  pstAccel->x, pstAccel->y, pstAccel->z,
							  pstMag->x, pstMag->y, pstMag->z );
							  */

	pstCxt->stRotation.x = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
	pstCxt->stRotation.y = -asin(2.0f * (q1 * q3 - q0 * q2));
	pstCxt->stRotation.z = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);

#endif

	return;
}

/* ************************************************************************** */
static float GetMag( float x, float y, float z )
{
	return sqrtf( x*x + y*y + z*z );
}
