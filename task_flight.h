#ifndef TASK_FLIGHT_H
#define TASK_FLIGHT_H

#include "ledstat.h"		// Status led pattern controller

/**
 * @brief		Initialises the flight task.
 */
void TASK_FLIGHT_Create( stLEDSTAT_Ctx_t *const pstLedStat );

#endif
