#ifndef LEDSTAT_H
#define LEDSTAT_H

#include <stdint.h>			// std types
#include <stdbool.h>		// bool definition
#include <stddef.h>			// size_t

#define LEN_PATTERN_MAX		( 16 )

typedef void (*pfnLEDSTAT_SetLed)( void *const pvUserState, const bool bState );

typedef struct
{
	pfnLEDSTAT_SetLed pfnSetLed;
	uint16_t uiIndex;
	uint16_t uiTimer;
	uint16_t auiPattern[ LEN_PATTERN_MAX ];
	size_t sLen;
	void *pvUserState;

} stLEDSTAT_Ctx_t;

/**
 * @brief		Initialises the context of an LEDSTAT module.
 * @param[in]	pstCtx		The ledstat context to initialise.
 * @param[in]	pfnSetLed	Function pointer to set/clear the led.
 * @param[in]	pvUserState	The state to pass back to the set led callback.
 */
void LEDSTAT_Create( stLEDSTAT_Ctx_t *const pstCtx, pfnLEDSTAT_SetLed pfnSetLed, void *const pvUserState );

/**
 * @brief		Configures the pattern.
 * @param[in]	pstCtx		The ledstat context to use.
 * @param[in]	pauiPattern	Pointer to the start of the new pattern.
 * @param[in]	sLen		Length of the new pattern.
 * @return		0 on success, -1 on error.
 */
int LEDSTAT_SetPattern( stLEDSTAT_Ctx_t *const pstCtx, const uint16_t *pauiPattern, const size_t sLen );

/**
 * @brief		Processes the pattern. This can be called as often as you like
 * 				as long as the passed timestep is accurate, however the faster
 * 				it is called the better the resolution the patterns will be.
 * @param[in]	pstCtx		The ledstat context to use.
 * @param[in]	uiTimestep	Time since the last call to this function in millis.
 */
void LEDSTAT_Process( stLEDSTAT_Ctx_t *const pstCtx, const uint16_t uiTimestep );

#endif
