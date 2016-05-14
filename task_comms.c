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
#include "FreeRTOSConfig.h"
#include "timers.h"
#include "portmacro.h"

#include "config.h"			// Board specific config
#include "SFE_LSM9DS0.h"	// LSM9DS0 driver
#include "ledstat.h"		// Status led pattern controller
#include "flight.h"			// Flight controller
#include "vector3f.h"		// vector3f_t
#include "io_driver.h"		// IO driver
#include "SFE_LSM9DS0.h"	// LSM9DS0 driver
#include "i2c.h"			// I2C device driver

/* ************************************************************************** **
 * Macros and Defines
 * ************************************************************************** */
#define TASK_TICK_MS		( 100UL )
#define mArraySize( x )		( sizeof( x ) / sizeof( x[0] ) )

#define LSM9DS0_XM				( 0x1D ) // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G				( 0x6B ) // Would be 0x6A if SDO_G is LOW

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

/* ************************************************************************** **
 * Local Variables
 * ************************************************************************** */
static TaskHandle_t xCommsTaskHandle = NULL;
static TimerHandle_t xCommsTimerHandle = NULL;

/* ************************************************************************** **
 * API Functions
 * ************************************************************************** */

/* ************************************************************************** */
void TASK_COMMS_Create( void )
{
	// Create our flight task
	xTaskCreate( TaskHandler,					// The task's callback function
				 "TASK_Comms",					// Task name
				 configMINIMAL_STACK_SIZE,		// Just use the minimal stack size for now...
				 NULL,							// Parameter to pass to the callback function, we have nothhing to pass..
				 2,								// Priority, this is our only task so.. lets just use 0
				 &xCommsTaskHandle );			// We could put a pointer to a task handle here which will be filled in when the task is created

	// Create a timer for our flight task
	xCommsTimerHandle = xTimerCreate( "TIMER_Comms", 							// A text name, purely to help debugging.
						( TASK_TICK_MS / portTICK_PERIOD_MS ),	// The timer period, in this case 500ms.
						pdTRUE,										// We want this to be a recurring timer so set uxAutoReload to pdTRUE.
						( void * ) 0,								// The ID is not used, so can be set to anything.
						TimerHandler );								// The callback function that is called when the timer expires.

	return;
}

/* ************************************************************************** **
 * Local Functions
 * ************************************************************************** */

/* ************************************************************************** */
static void TaskHandler( void *arg )
{
	// Start our timer which will resume this task accurately on a tick
	xTimerStart( xCommsTimerHandle, 0 );

	for ( ; ; )
	{
		printf( "Round again\r\n" );

		// Suspend until our timer wakes us up again
		vTaskSuspend( NULL );
	}
}

/* ************************************************************************** */
static void TimerHandler( TimerHandle_t xTimer )
{
	vTaskResume( xCommsTaskHandle );
}
