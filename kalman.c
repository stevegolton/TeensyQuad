/**
 * Kalman filter (just in the x axis for now).
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>		// <- TODO how efficient is this - used for atan2() (see below)?
#include <kalman.h>

#include <vector3f.h>

#define PI					( 3.14159265359f )
#define RAD2DEG				( 180/PI )
#define fRatioGyro			( 0.98 )
#define fRatioAccel			( 1 - fRatioGyro )

static float GetMag( float x, float y, float z );

/* ************************************************************************** */
void KALMAN_Setup( stKALMAN_Cxt_t *pstCxt )
{
	/* We will set the variables like so, these can also be tuned by the user */
	pstCxt->Q_angle = 0.001f;
	pstCxt->Q_bias = 0.003f;
	pstCxt->R_measure = 0.03f;

	pstCxt->angle = 0.0f; // Reset the angle
	pstCxt->bias = 0.0f; // Reset bias

	pstCxt->P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
	pstCxt->P[0][1] = 0.0f;
	pstCxt->P[1][0] = 0.0f;
	pstCxt->P[1][1] = 0.0f;
}

/* ************************************************************************** */
float KALMAN_Update( stKALMAN_Cxt_t *pstCxt,
						   float newRate,
						   float newAngle,
						   float dt )
{
	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	pstCxt->rate = newRate - pstCxt->bias;
	pstCxt->angle += dt * pstCxt->rate;

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	pstCxt->P[0][0] += dt * (dt*pstCxt->P[1][1] - pstCxt->P[0][1] - pstCxt->P[1][0] + pstCxt->Q_angle);
	pstCxt->P[0][1] -= dt * pstCxt->P[1][1];
	pstCxt->P[1][0] -= dt * pstCxt->P[1][1];
	pstCxt->P[1][1] += pstCxt->Q_bias * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	float S = pstCxt->P[0][0] + pstCxt->R_measure; // Estimate error
	/* Step 5 */
	float K[2]; // Kalman gain - This is a 2x1 vector
	K[0] = pstCxt->P[0][0] / S;
	K[1] = pstCxt->P[1][0] / S;

	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	float y = newAngle - pstCxt->angle; // Angle difference
	/* Step 6 */
	pstCxt->angle += K[0] * y;
	pstCxt->bias += K[1] * y;

	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	float P00_temp = pstCxt->P[0][0];
	float P01_temp = pstCxt->P[0][1];

	pstCxt->P[0][0] -= K[0] * P00_temp;
	pstCxt->P[0][1] -= K[0] * P01_temp;
	pstCxt->P[1][0] -= K[1] * P00_temp;
	pstCxt->P[1][1] -= K[1] * P01_temp; ;

	return pstCxt->angle;
}

/* ************************************************************************** */
static float GetMag( float x, float y, float z )
{
	return sqrtf( x*x + y*y + z*z );
}
