#include <pid.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

/* ************************************************************************** */
void PID_Setup( stPidCxt_t *pstCxt,
				float fKInt,
				float fKProp,
				float fKDiff,
				float fIntMax,
				float fIntMin )
{
	// Cache local values
	pstCxt->fKInt = fKInt;
	pstCxt->fKProp = fKProp;
	pstCxt->fKDiff = fKDiff;
	pstCxt->fIntMax = fIntMax;
	pstCxt->fIntMin = fIntMin;

	pstCxt->fLastError = 0;
	pstCxt->fIntegral = 0;
}

/* ************************************************************************** */
float PID_Update( stPidCxt_t *pstCxt, float fError, float fTimestep_s )
{
	float fDiff;

	// Calc integral
	pstCxt->fIntegral += ( fError * fTimestep_s );

	// Integral Limiting
	if ( pstCxt->fIntegral > pstCxt->fIntMax )
	{
		pstCxt->fIntegral = pstCxt->fIntMax;
	}
	else if ( pstCxt->fIntegral < pstCxt->fIntMin )
	{
		pstCxt->fIntegral = pstCxt->fIntMin;
	}

	// Calc differential
	fDiff = ( fError - pstCxt->fLastError ) / fTimestep_s;
	pstCxt->fLastError = fError;

	// Calc output
	return ( fError * pstCxt->fKProp ) + ( pstCxt->fIntegral * pstCxt->fKInt ) + ( fDiff * pstCxt->fKDiff );
}

/* ************************************************************************** */
void PID_SetGains( stPidCxt_t *pstCxt, const float fP, const float fD, const float fI )
{
	pstCxt->fKInt = fI;
	pstCxt->fKProp = fP;
	pstCxt->fKDiff = fD;
}
