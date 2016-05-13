#include "ledstat.h"

#include <stdint.h>			// std types
#include <stdbool.h>		// bool definition
#include <stddef.h>			// size_t

void LEDSTAT_Create( stLEDSTAT_Ctx_t *const pstCtx, pfnLEDSTAT_SetLed pfnSetLed, void *const pvUserState )
{
	pstCtx->pfnSetLed = pfnSetLed;
	pstCtx->pvUserState = pvUserState;
	pstCtx->uiIndex = 0;
	pstCtx->uiTimer = 0;
	pstCtx->sLen = 0;

	// Clear LED initially
	pstCtx->pfnSetLed( pstCtx->pvUserState, false );
}

int LEDSTAT_SetPattern( stLEDSTAT_Ctx_t *const pstCtx, const uint16_t *pauiPattern, const size_t sLen )
{
	if ( sLen <= LEN_PATTERN_MAX )
	{
		// Immediately set the pattern length to 0 multithreaded calls to
		// the processing function will return without doing anything.
		// This is sort of a poor man's locking and will only work on systems
		// where copying a size_t integer is an atomic operation!
		pstCtx->sLen = 0;
		pstCtx->uiIndex = 0;
		pstCtx->uiTimer = 0;

		// Copy the pattern across to our private structure
		memcpy( pstCtx->auiPattern, pauiPattern, sLen * sizeof( uint16_t ) );
		pstCtx->sLen = sLen;

		return 0;
	}
	else
	{
		return -1;
	}
}

void LEDSTAT_Process( stLEDSTAT_Ctx_t *const pstCtx, const uint16_t uiTimestep )
{
	// If the sequence length is 0, simply get out
	if ( 0 == pstCtx->sLen )
	{
		return;
	}

	// Increment the timer by the timestep amount
	pstCtx->uiTimer += uiTimestep;

	// Reset the timer on sequence element overflow
	if ( pstCtx->uiTimer >= pstCtx->auiPattern[ pstCtx->uiIndex ] )
	{
		pstCtx->uiTimer = 0;
		pstCtx->uiIndex++;

		// Reset the index on pattern overflow
		if ( pstCtx->sLen == pstCtx->uiIndex )
		{
			pstCtx->uiIndex = 0;
		}

		// Set the LED depending on where we are in the sequence
		if ( 0 == ( pstCtx->uiIndex % 2 ) )
		{
			// Set LED
			pstCtx->pfnSetLed( pstCtx->pvUserState, true );
		}
		else
		{
			// Clear LED
			pstCtx->pfnSetLed( pstCtx->pvUserState, false );
		}
	}

	return;
}
