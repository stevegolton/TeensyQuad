#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
	float fKInt;
	float fKProp;
	float fKDiff;

	float fIntMax;
	float fIntMin;

	float fLastError;
	float fIntegral;

} stPidCxt_t;

void PID_Setup( stPidCxt_t *pstCxt,
				float fKInt,
				float fKProp,
				float fKDiff,
				float fIntMax,
				float fIntMin );
float PID_Update( stPidCxt_t *pstCxt, float fError, float fTimestep_s );
void PID_SetGains( stPidCxt_t *pstCxt, const float fP, const float fD, const float fI );

#endif
