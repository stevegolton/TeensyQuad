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

#include "config.h"			// Board specific config
#include "vector3f.h"		// vector3f_t
#include "io_driver.h"		// IO driver
#include "IPC_types.h"		// stFlightDetails_t
#include "mavlink_1.0/common/mavlink.h"	// Mavlink msg writing & parsing

// This is cheeky - we should be able to go through the stdio interface!
extern int port_getchar(void);
extern int port_putchar( int c );

/* ************************************************************************** **
 * Macros and Defines
 * ************************************************************************** */
#define TASK_TICK_MS		( 20UL )
#define mArraySize( x )		( sizeof( x ) / sizeof( x[0] ) )

#define SYSID				( 69 )		// hehe
#define PI					( 3.14159265359f )
#define RAD2DEG				( 180 / PI )
#define NUM_PARAMS			( sizeof( params )/ sizeof(struct parameter_element) )

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

static void SendHeartbeat( void );
static void SendAttitude( void );

/* ************************************************************************** **
 * Local Variables
 * ************************************************************************** */
static TaskHandle_t xCommsTaskHandle = NULL;
static TimerHandle_t xCommsTimerHandle = NULL;

static const mavlink_system_t mavlink_system =
{
	.sysid = SYSID,
	.compid = MAV_COMP_ID_ALL
};
static const uint8_t system_type = MAV_TYPE_QUADROTOR;
static const uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
static const uint8_t system_mode = MAV_MODE_STABILIZE_ARMED;
static const uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
static const uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
static mavlink_message_t mavlink_mesg_rx;
static mavlink_message_t mavlink_mesg_tx;

static uint32_t uiMillisSinceBoot;
static QueueHandle_t _xCommsQueue;

/* ************************************************************************** **
 * API Functions
 * ************************************************************************** */

/* ************************************************************************** */
void TASK_COMMS_Create( QueueHandle_t xCommsQueue )
{
	_xCommsQueue = xCommsQueue;

	// Create our flight task
	xTaskCreate( TaskHandler,					// The task's callback function
				 "TASK_Comms",					// Task name
				 300,							// We need a relatively big stack for mavlink message creation!
				 NULL,							// Parameter to pass to the callback function, we have nothhing to pass..
				 2,								// Priority, this is our only task so.. lets just use 0
				 &xCommsTaskHandle );			// We could put a pointer to a task handle here which will be filled in when the task is created

	// Create a timer for our flight task
	xCommsTimerHandle = xTimerCreate( "TIMER_Comms", 							// A text name, purely to help debugging.
						( TASK_TICK_MS / portTICK_PERIOD_MS ),	// The timer period, in this case 500ms.
						pdTRUE,										// We want this to be a recurring timer so set uxAutoReload to pdTRUE.
						( void * ) 0,								// The ID is not used, so can be set to anything.
						TimerHandler );								// The callback function that is called when the timer expires.

	uiMillisSinceBoot = 0;

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
		uiMillisSinceBoot += TASK_TICK_MS;

		SendHeartbeat();
		SendAttitude();

		// Suspend until our timer wakes us up again
		vTaskSuspend( NULL );
	}
}

/* ************************************************************************** */
static void TimerHandler( TimerHandle_t xTimer )
{
	vTaskResume( xCommsTaskHandle );
}

/* ************************************************************************** */
static void SendHeartbeat( void )
{
	// Initialize the required buffers
	uint16_t len;
	uint16_t uiIndex = 0;

	// Pack the message
	mavlink_msg_heartbeat_pack( mavlink_system.sysid,
								mavlink_system.compid,
								&mavlink_mesg_tx,
								system_type,
								autopilot_type,
								system_mode,
								custom_mode,
								system_state);

	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer( buf, &mavlink_mesg_tx );

	while( uiIndex < len )
	{
		port_putchar( buf[uiIndex++] );
	}
}

/* ************************************************************************** */
static void SendAttitude( void )
{
	uint16_t len;
	stFlightDetails_t stFlightDetails;
	uint16_t uiIndex = 0;

	// Clear the queue
	while ( uxQueueMessagesWaiting( _xCommsQueue ) )
	{
		xQueueReceive( _xCommsQueue, &stFlightDetails, 0 );
	}

	// Send attitude - negate the roll (x) value for qground control
	mavlink_msg_attitude_pack( mavlink_system.sysid,
							   mavlink_system.compid,
							   &mavlink_mesg_tx,
							   uiMillisSinceBoot,
							   - stFlightDetails.stAttitude.x / RAD2DEG,
							   stFlightDetails.stAttitude.y / RAD2DEG,
							   stFlightDetails.stAttitude.z / RAD2DEG,
							   -stFlightDetails.stAttitudeRate.x / RAD2DEG,
							   stFlightDetails.stAttitudeRate.y / RAD2DEG,
							   stFlightDetails.stAttitudeRate.z / RAD2DEG
							   );

	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer( buf, &mavlink_mesg_tx );

	// Send the message
	while( uiIndex < len )
	{
		port_putchar( buf[uiIndex++] );
	}
}

