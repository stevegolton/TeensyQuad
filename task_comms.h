#ifndef TASK_COMMS_H
#define TASK_COMMS_H

#include "FreeRTOS.h"		// FreeRTOS
#include "FreeRTOSConfig.h"	// FreeRTOS portable config
#include "portmacro.h"		// Portable functions
#include "timers.h"			// FreeRTOS timers
#include "queue.h"			// FreeRTOS queues

/**
 * @brief		Initialises the flight task.
 */
void TASK_COMMS_Create( void );

#endif
