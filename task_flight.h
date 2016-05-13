#ifndef TASK_FLIGHT_H
#define TASK_FLIGHT_H

#include "SFE_LSM9DS0.h"	// LSM9DS0 driver
#include "ledstat.h"		// Status led pattern controller

/**
 * @brief		Initialises the flight task.
 */
void TASK_FLIGHT_Create( stLSM9DS0_t *const pstImu, stLEDSTAT_Ctx_t *const pstLedStat );

#endif
