#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>		// <- TODO how efficient is this - used for atan2() (see below)?
#include <string.h>

#include "sensor_fusion.h"
#include "kalman.h"
#include "vector3f.h"

#define PI					( 3.14159265359f )
#define RAD2DEG				( 180/PI )
#define fRatioGyro			( 0.98 )
#define fRatioAccel			( 1 - fRatioGyro )

static float GetMag( float x, float y, float z );

/* ************************************************************************** */
void SENSORFUSION_Setup( stSENSORFUSION_Cxt_t *pstCxt )
{
	// Reset rotation vector
	pstCxt->stRotation.x = 0;
	pstCxt->stRotation.y = 0;
	pstCxt->stRotation.z = 0;

	KALMAN_Setup( &pstCxt->stKalmanPitch );
	KALMAN_Setup( &pstCxt->stKalmanRoll );
}

/* ************************************************************************** */
void SENSORFUSION_Update( stSENSORFUSION_Cxt_t *pstCxt,
						  vector3f_t *pstGyro,
						  vector3f_t *pstAccel,
						  vector3f_t *pstRotation,
						  float fTimestep_s )
{
	float pitchRate = pstGyro->y;
	float pitchAngle = atan2f( -pstAccel->x, GetMag( pstAccel->z, pstAccel->y, 0 ) );
	float rollRate = pstGyro->x;
	float rollAngle = atan2f( pstAccel->y, GetMag( pstAccel->z, pstAccel->x, 0 ) );

#if 0
	pstCxt->stRotation.y = KALMAN_Update( &pstCxt->stKalmanPitch, pitchRate, pitchAngle, fTimestep_s );
	pstCxt->stRotation.x = KALMAN_Update( &pstCxt->stKalmanRoll, rollRate, rollAngle, fTimestep_s );
	pstCxt->stRotation.z = pstGyro->z * fTimestep_s + pstCxt->stRotation.z;

	memcpy( pstRotation, &pstCxt->stRotation, sizeof( vector3f_t ) );
#else
	// Complimentary filter
	pstCxt->stRotation.x = ( fRatioGyro * ( pstGyro->x * fTimestep_s + pstCxt->stRotation.x ) )
			+ ( fRatioAccel * ( atan2f( pstAccel->y, GetMag( pstAccel->z, pstAccel->x, 0 ) ) ) );
	pstCxt->stRotation.y = ( fRatioGyro * ( pstGyro->y * fTimestep_s + pstCxt->stRotation.y ) )
			- ( fRatioAccel * ( atan2f( pstAccel->x, GetMag( pstAccel->z, pstAccel->y, 0 ) ) ) );

	memcpy( pstRotation, &pstCxt->stRotation, sizeof( vector3f_t ) );

#endif

	return;
}

/* ************************************************************************** */
static float GetMag( float x, float y, float z )
{
	return sqrtf( x*x + y*y + z*z );
}
