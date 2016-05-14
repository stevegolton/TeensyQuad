#ifndef TASK_LED_H
#define TASK_LED_H

#include "FreeRTOS.h"		// FreeRTOS
#include "FreeRTOSConfig.h"	// FreeRTOS portable config
#include "portmacro.h"		// Portable functions
#include "timers.h"			// FreeRTOS timers
#include "queue.h"			// FreeRTOS queues

/**
 * @brief		Initialises the ledstat task.
 */
void TASK_LED_Create( QueueHandle_t xLedPatternQueue );

#endif
