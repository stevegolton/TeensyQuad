/* ************************************************************************** **
 * Includes
 * ************************************************************************** */
#include "task_comms.h"

#include <stdint.h>			// std types
#include <stdbool.h>		// bool definition
#include <stddef.h>			// size_t
#include <string.h>			// memset & friends
#include <stdio.h>			// printf & friends

#include "FreeRTOS.h"		// FreeRTOS
#include "FreeRTOSConfig.h"	// FreeRTOS portable config
#include "portmacro.h"		// Portable functions
#include "timers.h"			// FreeRTOS timers
#include "queue.h"			// FreeRTOS queues

#include "io_driver.h"		// IO driver
#include "IPC_types.h"		// stLedPattern_t
#include "ledstat.h"		// Led status controller

/* ************************************************************************** **
 * Macros and Defines
 * ************************************************************************** */
#define TASK_TICK_MS		( 100UL )
#define mArrayLen( x )		( sizeof( x ) / sizeof( x[0] ) )

/* ************************************************************************** **
 * Typedefs
 * ************************************************************************** */

/* ************************************************************************** **
 * Function Prototypes
 * ************************************************************************** */
/**
 * @brief		Entry point for the fligh task.
 * @param[in]	arg		Opaque pointer to user data.
 */
static void TaskHandler( void *arg );

/**
 * @brief		Callback for the timer.
 * @param[in]	xTimer	Timer handle.
 */
static void TimerHandler( TimerHandle_t xTimer );

static void SetLed( void *const pvUserState, const bool bState );

/* ************************************************************************** **
 * Local Variables
 * ************************************************************************** */
static TaskHandle_t xLedTaskHandle = NULL;
static TimerHandle_t xLedTimerHandle = NULL;
static QueueHandle_t _xLedPatternQueue;
static stLEDSTAT_Ctx_t stLedStat;

static const uint16_t auiDefaultPattern[] = { 200, 500 };

/* ************************************************************************** **
 * API Functions
 * ************************************************************************** */

/* ************************************************************************** */
void TASK_LED_Create( QueueHandle_t xLedPatternQueue )
{
	_xLedPatternQueue = xLedPatternQueue;

	LEDSTAT_Create( &stLedStat, SetLed, NULL );

	// Create our flight task
	xTaskCreate( TaskHandler,					// The task's callback function
				 "TASK_Led",					// Task name
				 300,							// We need a relatively big stack for mavlink message creation!
				 NULL,							// Parameter to pass to the callback function, we have nothhing to pass..
				 2,								// Priority, this is our only task so.. lets just use 0
				 &xLedTaskHandle );				// We could put a pointer to a task handle here which will be filled in when the task is created

	// Create a timer for our flight task
	xLedTimerHandle = xTimerCreate( "TIMER_Led",					// A text name, purely to help debugging.
						( TASK_TICK_MS / portTICK_PERIOD_MS ),		// The timer period, in this case 500ms.
						pdTRUE,										// We want this to be a recurring timer so set uxAutoReload to pdTRUE.
						( void * ) 0,								// The ID is not used, so can be set to anything.
						TimerHandler );								// The callback function that is called when the timer expires.

	// Set the default pattern
	LEDSTAT_SetPattern( &stLedStat, auiDefaultPattern, mArrayLen( auiDefaultPattern ) );

	return;
}

/* ************************************************************************** **
 * Local Functions
 * ************************************************************************** */

/* ************************************************************************** */
static void TaskHandler( void *arg )
{
	stLedPattern_t stLedPattern;

	// Start our timer which will resume this task accurately on a tick
	xTimerStart( xLedTimerHandle, 0 );

	for ( ; ; )
	{
		// Clear the queue
		while ( uxQueueMessagesWaiting( _xLedPatternQueue ) )
		{
			xQueueReceive( _xLedPatternQueue, &stLedPattern, 0 );
			LEDSTAT_SetPattern( &stLedStat, stLedPattern.auiPattern, stLedPattern.sPatternLen );
		}

		LEDSTAT_Process( &stLedStat, TASK_TICK_MS );

		// Suspend ourselves until some nice person resumes us...
		vTaskSuspend( NULL );
	}

	return;
}

/* ************************************************************************** */
static void TimerHandler( TimerHandle_t xTimer )
{
	vTaskResume( xLedTaskHandle );

	return;
}

/* ************************************************************************** */
static void SetLed( void *const pvUserState, const bool bState )
{
	// FIXME Should not be accessing registers directly from within a task,
	// access should be wrapped in an LDD!
	if ( true == bState )
	{
		// Set LED
		GPIOC_PSOR = ( 1 << 5 );
	}
	else
	{
		// Clear LED
		GPIOC_PCOR = ( 1 << 5 );
	}

	return;
}

