#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include <vector3f.h>

typedef struct
{
	float Q_angle; // Process noise variance for the accelerometer
	float Q_bias; // Process noise variance for the gyro bias
	float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

	float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
	float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	float P[2][2]; // Error covariance matrix - This is a 2x2 matrix

} stKALMAN_Cxt_t;

void KALMAN_Setup( stKALMAN_Cxt_t *pstCxt );
float KALMAN_Update( stKALMAN_Cxt_t *pstCxt,
						   float newRate,
						   float newAngle,
						   float dt );

#endif
