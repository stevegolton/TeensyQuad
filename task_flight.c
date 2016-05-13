#include "task_flight.h"

#include <stdint.h>			// std types
#include <stdbool.h>		// bool definition
#include <stddef.h>			// size_t

#include "FreeRTOS.h"		// FreeRTOS
#include "FreeRTOSConfig.h"
#include "timers.h"
#include "portmacro.h"

#include "SFE_LSM9DS0.h"	// LSM9DS0 driver
#include "ledstat.h"		// Status led pattern controller
#include "flight.h"			// Flight controller
#include "vector3f.h"		// vector3f_t

#define FLIGHT_TICK_MS			( 10UL )

typedef enum
{
	eStateNone,
	eStateCal,
	eStateFlight,

} eState_t;

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

/**
 * @brief		Prints some debug to stdout.
 * @param[in]	accel	Current accelerometer values.
 * @param[in]	accel	Current gyro values.
 */
static void PrintDebug( vector3f_t accel, vector3f_t gyro );

static TaskHandle_t xFlightTaskHandle = NULL;
static TimerHandle_t xFlightTimerHandle = NULL;
static stLSM9DS0_t *_pstImu;
static stLEDSTAT_Ctx_t *_pstLedStat;
static eState_t eState;

static const uint16_t auiLedPatternCal[] = { 100, 300 };
static const uint16_t auiLedPatternFlight[] = { 500, 500 };

void TASK_FLIGHT_Create( stLSM9DS0_t *const pstImu, stLEDSTAT_Ctx_t *const pstLedStat )
{
	// Store the instances of the IMU and ledstat modules to use
	_pstImu = pstImu;
	_pstLedStat = pstLedStat;

	// Create our flight task
	xTaskCreate( TaskHandler,					// The task's callback function
				 "TASK_Flight",					// Task name
				 500,							// Make the flight controller's stack large as we are likely to do many function calls
				 NULL,							// Parameter to pass to the callback function, we have nothhing to pass..
				 2,								// Priority, this is our only task so.. lets just use 0
				 &xFlightTaskHandle );			// We could put a pointer to a task handle here which will be filled in when the task is created

	// Create a timer for our flight task
	xFlightTimerHandle = xTimerCreate( "TIMER_Flight", 							// A text name, purely to help debugging.
									   ( FLIGHT_TICK_MS / portTICK_PERIOD_MS ),	// The timer period, in this case 500ms.
									   pdTRUE,										// We want this to be a recurring timer so set uxAutoReload to pdTRUE.
									   ( void * ) 0,								// The ID is not used, so can be set to anything.
									   TimerHandler );								// The callback function that is called when the timer expires.

	// Initialise the state to "none"
	eState = eStateNone;

	return;
}

static void TaskHandler( void *arg )
{
	vector3f_t accel;
	vector3f_t gyro;
	int iResult;

	// Start our timer which will resume this task accurately on a tick
	xTimerStart( xFlightTimerHandle, 0 );

	for ( ; ; )
	{
		// Read the latest accel and gyro values
		LSM9DS0_readAccel( _pstImu );
		LSM9DS0_readGyro( _pstImu );

		// Scale the accel values into g's
		accel.x = LSM9DS0_calcAccel( _pstImu, _pstImu->ax );
		accel.y = LSM9DS0_calcAccel( _pstImu, _pstImu->ay );
		accel.z = LSM9DS0_calcAccel( _pstImu, _pstImu->az );

		// Scale the gyro values into rad/sec
		gyro.x = LSM9DS0_calcGyro( _pstImu, _pstImu->gx );
		gyro.y = LSM9DS0_calcGyro( _pstImu, _pstImu->gy );
		gyro.z = LSM9DS0_calcGyro( _pstImu, _pstImu->gz );

		// Process flight controller
		iResult = flight_process( FLIGHT_TICK_MS, accel, gyro );

		// Process state machine
		switch( eState )
		{
			case eStateNone:
			{
				eState = eStateCal;
				LEDSTAT_SetPattern( _pstLedStat, auiLedPatternCal, sizeof( auiLedPatternCal ) / sizeof( auiLedPatternCal[0] ) );
				break;
			}

			case eStateCal:
			{
				if ( 1 == iResult )
				{
					eState = eStateFlight;
					LEDSTAT_SetPattern( _pstLedStat, auiLedPatternFlight, sizeof( auiLedPatternFlight ) / sizeof( auiLedPatternFlight[0] ) );
				}
				break;
			}

			case eStateFlight:
			default:
			{
				// Do nothing
				break;
			}
		}

		PrintDebug( accel, gyro );

		// Suspend until our timer wakes us up again
		vTaskSuspend( NULL );
	}
}

static void TimerHandler( TimerHandle_t xTimer )
{
	vTaskResume( xFlightTaskHandle );
}

static void PrintDebug( vector3f_t accel, vector3f_t gyro )
{
	// Don't print debug!
#if 0
	// Print accel and gyro to stdout - values are * 1000 */
	printf( "Accel = %d, %d, %d\r\n",
			(int)( accel.x * 1000 ),
			(int)( accel.y * 1000 ),
			(int)( accel.z * 1000 ) );

	printf( "Gyro = %d, %d, %d\r\n",
			(int)( gyro.x * 1000 ),
			(int)( gyro.y * 1000 ),
			(int)( gyro.z * 1000 ) );

	// Print receiver inputs to stdout
	for( i = 0; i < RECEIVER_NUM_CHAN_IN; i ++ )
	{
		IODRIVER_GetInputPulseWidth( i, &inputs[i] );
		printf( "Receiver input %d = %d\r\n", i, (int)inputs[i] );
	}
#endif
}

