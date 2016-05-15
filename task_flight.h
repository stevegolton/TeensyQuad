#ifndef TASK_FLIGHT_H
#define TASK_FLIGHT_H

#include "FreeRTOS.h"		// FreeRTOS
#include "FreeRTOSConfig.h"	// FreeRTOS portable config
#include "portmacro.h"		// Portable functions
#include "timers.h"			// FreeRTOS timers
#include "queue.h"			// FreeRTOS queues

#include "ledstat.h"		// Status led pattern controller

/**
 * @brief		Initialises the flight task.
 */
void TASK_FLIGHT_Create( void );

#endif
