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
#include "params.h"			// System parameters

// This is horrible - we should be able to go through the stdio interface..
// perhaps through a file descriptor? Need to look up how uarts are mapped
// from embedded platforms.
extern int port_getchar( void );
extern int port_putchar( int c );
extern int port_getchar_nonblock( void );

/* ************************************************************************** **
 * Macros and Defines
 * ************************************************************************** */
#define TASK_TICK_MS		( 50UL )
#define mArraySize( x )		( sizeof( x ) / sizeof( x[0] ) )

#define SYSID				( 69 )		// hehe
#define PI					( 3.14159265359f )
#define RAD2DEG				( 180 / PI )

//#define PID_TUNE

/* ************************************************************************** **
 * Typedefs
 * ************************************************************************** */

typedef struct
{
	uint8_t uiSig1;
	uint8_t uiSig2;
	uint16_t uiLen;

	uint16_t uiRx1;
	uint16_t uiRx2;
	uint16_t uiRx3;
	uint16_t uiRx4;
	float fM1;
	float fM2;
	float fM3;
	float fM4;
	float fAttitudeRoll;
	float fAttitudePitch;
	float fAttitudeYaw;
	uint32_t uiRxMsgCnt;
	float fGainRateP;

} stPidTuneMsgFmt_t;

typedef struct
{
	uint8_t uiSig1;
	uint8_t uiSig2;
	uint16_t uiLen;

	float fPidAngleP;
	float fPidAngleI;
	float fPidAngleD;
	float fPidRateP;
	float fPidRateI;
	float fPidRateD;

} stPidTuneCfgMsgFmt_t;

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

static void SendPIDTuneTelem( void );
static void ReadPIDTuneMessage( void );

static void SendHeartbeat( void );
static void SendAttitude( void );
static void ReadMavlink( void );
static void SendParam( int iParamIndex );

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
static uint16_t uiParamIndex;

static stPARAM_t *pstPidGainRateP;
static stPARAM_t *pstPidGainRateD;
static stPARAM_t *pstPidGainAngleP;
static stPARAM_t *pstPidGainRateYawP;
static stPARAM_t *pstPidGainRateYawD;

static uint8_t bySig1;
static uint8_t bySig2;
static uint32_t uiRxMsgCount;
static stPidTuneCfgMsgFmt_t stMsg;
static int uiIndex;

/* ************************************************************************** **
 * API Functions
 * ************************************************************************** */

/* ************************************************************************** */
void TASK_COMMS_Create( QueueHandle_t xCommsQueue )
{
	_xCommsQueue = xCommsQueue;

	bySig1 = bySig2 = 0;
	uiRxMsgCount = 0;
	uiIndex = ( sizeof( stPidTuneCfgMsgFmt_t ) - 2);

	// Create our flight task
	xTaskCreate( TaskHandler,					// The task's callback function
				 "TASK_Comms",					// Task name
				 512,							// We need a relatively big stack for mavlink message creation!
				 NULL,							// Parameter to pass to the callback function, we have nothhing to pass..
				 1,								// Priority, this is our only task so.. lets just use 0
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

	int value;
	int count = 0;
	char buf[100];
	int index;

	uiParamIndex = PARAM_GetParamCount();

	pstPidGainRateP = PARAM_FindParamByName( "PIDGainRate_P", 0, NULL );
	pstPidGainRateD = PARAM_FindParamByName( "PIDGainRate_D", 0, NULL );
	pstPidGainAngleP = PARAM_FindParamByName( "PIDGainAngle_P", 0, NULL );
	pstPidGainRateYawP = PARAM_FindParamByName( "PIDGainRateYaw_P", 0, NULL );
	pstPidGainRateYawD = PARAM_FindParamByName( "PIDGainRateYaw_D", 0, NULL );

	for ( ; ; )
	{
		uiMillisSinceBoot += TASK_TICK_MS;

#ifdef PID_TUNE

		ReadPIDTuneMessage();
		SendPIDTuneTelem();

#else // Otherwise do mavlink
		// Read mavlink messages from our receive buffer
		//ReadMavlink();

		// Send standard telemetry & heartbeat messages
		//SendHeartbeat();
		//SendAttitude();

		if ( uiParamIndex < PARAM_GetParamCount() )
		{
			SendParam( uiParamIndex++ );
		}

#endif

		// Suspend until our timint paramIndex;er wakes us up again
		vTaskSuspend( NULL );
	}
}

/* ************************************************************************** */
static void TimerHandler( TimerHandle_t xTimer )
{
	vTaskResume( xCommsTaskHandle );
}

/* ************************************************************************** */
static void SendPIDTuneTelem( void )
{
	stPidTuneMsgFmt_t stMsg;
	stFlightDetails_t stFlightDetails;
	uint16_t uiIndex = 0;
	uint8_t *pbyMsg;

	memset( &stMsg, 0, sizeof( stPidTuneMsgFmt_t ) );

	// Clear the queue
	while ( uxQueueMessagesWaiting( _xCommsQueue ) )
	{
		xQueueReceive( _xCommsQueue, &stFlightDetails, 0 );
	}

	stMsg.fAttitudeRoll = stFlightDetails.stAttitude.x;
	stMsg.fAttitudePitch = stFlightDetails.stAttitude.y;
	stMsg.fAttitudeYaw = stFlightDetails.stAttitude.z;

	stMsg.uiSig1 = 0xA5;
	stMsg.uiSig2 = 0xA5;

	stMsg.uiLen = sizeof( stPidTuneMsgFmt_t ) - 4;

	stMsg.uiRxMsgCnt = uiRxMsgCount;

	stMsg.fGainRateP = pstPidGainRateP->fValue;

	pbyMsg = (uint8_t*)(void*)&stMsg;

	while( uiIndex < sizeof( stPidTuneMsgFmt_t ) )
	{
		port_putchar( pbyMsg[uiIndex++] );
	}
}

/* ************************************************************************** */
static void ReadPIDTuneMessage( void )
{
	int iReadValue;
	uint8_t *pbyMsg;

	pbyMsg = ( (uint8_t*)(void*)&stMsg ) + 2;

	// Read a byte at a time from our stream interface
	while ( -1 != ( iReadValue = port_getchar_nonblock() ) )
	{
		if ( uiIndex == ( sizeof( stPidTuneCfgMsgFmt_t ) - 2 ) )
		{
			bySig2 = iReadValue;

			if ( ( bySig1 == 0xA5 ) && ( bySig2 == 0xA5 ) )
			{
				bySig1 = bySig2 = 0;
				uiIndex = 0;
			}
			else
			{
				bySig1 = bySig2;
			}
		}
		else
		{
			// We are in the middle of reading a message
			pbyMsg[uiIndex++] = (uint8_t)iReadValue;

			if ( uiIndex == ( sizeof( stPidTuneCfgMsgFmt_t ) - 2 ) )
			{
				// Message done
				pstPidGainRateP->fValue = stMsg.fPidRateP;
				pstPidGainRateD->fValue = stMsg.fPidRateD;
				pstPidGainAngleP->fValue = stMsg.fPidAngleP;

				uiRxMsgCount++;
			}
		}
	}
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
							   stFlightDetails.stAttitude.x,
							   -stFlightDetails.stAttitude.y,
							   stFlightDetails.stAttitude.z,
							   stFlightDetails.stAttitudeRate.x,
							   -stFlightDetails.stAttitudeRate.y,
							   stFlightDetails.stAttitudeRate.z
							   );

	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer( buf, &mavlink_mesg_tx );

	// Send the message
	while( uiIndex < len )
	{
		port_putchar( buf[uiIndex++] );
	}
}

/* ************************************************************************** */
static void ReadMavlink( void )
{
	mavlink_status_t stStatus;
	mavlink_param_set_t set;
	mavlink_param_request_read_t stMsgParamReqRead;
	int iReadValue;
	stPARAM_t *pstParam;
	size_t uiIndex;

	// Read a byte at a time from our stream interface
	while ( -1 != ( iReadValue = port_getchar_nonblock() ) )
	{
		// Try to get a new messagparamIndexe
		if( mavlink_parse_char( MAVLINK_COMM_0, (uint8_t)iReadValue, &mavlink_mesg_rx, &stStatus ) )
		{
			// Handle message
			switch( mavlink_mesg_rx.msgid )
			{
				case MAVLINK_MSG_ID_HEARTBEAT:
					// E.g. read GCS heartbeat and go into
					// comm lost mode if timer times out
					break;

				case MAVLINK_MSG_ID_COMMAND_LONG:
					// EXECUTE ACTION
					break;

				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:

					// Reset parameter index to start sending the params one by one
					uiParamIndex = 0;

					break;

				case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
				{
					mavlink_msg_param_request_read_decode( &mavlink_mesg_rx, &stMsgParamReqRead );

					SendParam( stMsgParamReqRead.param_index );

					break;
				}

				case MAVLINK_MSG_ID_PARAM_SET:
				{
					mavlink_msg_param_set_decode( &mavlink_mesg_rx, &set );

					pstParam = PARAM_FindParamByName( set.param_id, 16, &uiIndex );

					if ( pstParam )
					{
						// Only write and emit changes if there is actually a difference
						// AND only write if new value is NOT "not-a-number"
						// AND is NOT infinity
						if (    ( pstParam->fValue != set.param_value )
							 && ( !isnan( set.param_value ) )
							 && ( !isinf( set.param_value ) )
							 && ( set.param_type == MAV_PARAM_TYPE_REAL32 ) )
						{
							// Write new value
							pstParam->fValue = set.param_value;

							// Report back new value
							SendParam( uiIndex );
						}
						break;
					}
					break;
				}

				default:
					//Do nothing
					break;
			}
		}
	}
}

static void SendParam( int iParamIndex )
{
	uint16_t len;
	stPARAM_t *pastParamList = PARAM_GetParamList();
	uint16_t uiIndex = 0;

	mavlink_msg_param_value_pack( mavlink_system.sysid,
								  mavlink_system.compid,
								  &mavlink_mesg_tx,
								  pastParamList[ iParamIndex ].sName,
								  pastParamList[ iParamIndex ].fValue,
								  MAV_PARAM_TYPE_REAL32,
								  PARAM_GetParamCount(),
								  iParamIndex );

	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer( buf, &mavlink_mesg_tx );

	// Send the message
	while( uiIndex < len )
	{
		port_putchar( buf[uiIndex++] );
	}
}

