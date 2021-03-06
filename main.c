#include <stdio.h>			/* printf)_ & friends */
#include <stdbool.h>		// bool definition

#include "common.h"			/* uC specific dfns */
#include "FreeRTOS.h"		// FreeRTOS
#include "FreeRTOSConfig.h"	// FreeRTOS portable config
#include "portmacro.h"		// Portable functions
#include "timers.h"			// FreeRTOS timers
#include "queue.h"			// FreeRTOS queues

#include "i2c.h"			/* i2c ldd */
#include "uart.h"			/* uart ldd */
#include "vector3f.h"		/* vector3f_t */
#include "io_driver.h"
#include "ledstat.h"		/* Status led pattern controller */
#include "task_flight.h"	/* Initialises the flight task */
#include "task_comms.h"		/* Comms task */
#include "task_led.h"		/* Led task */
#include "IPC_types.h"		// stFlightDetails_t

// Define me if you want debugging, remove me for release!
//#define configASSERT( x )     if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

#define STARTUP_BLINK_COUNT		( 3 )
#define STARTUP_BLINK_PERIOD	( 100 )

/**
 * @brief		Delay using a loop. MillisecxFlightTimerHandleronds are very approximate based on
 * 				trial and error for our clock speed. Interrupts with slow this
 * 				down.
 * @param[in]	ms		Delay in ms.
 */
static __inline__ void dumbdelay_ms( const uint32_t ms )
{
	uint32_t loops;
	uint32_t index;

	// Calc delay in clock cycles
	loops = ms * ( (uint32_t)mcg_clk_hz / 10000 );

	// Dumb delay
	for ( index = 0; index < loops; index++ );
}

/*!
 * \brief	Called by the system when a hard fault is encountered.
 * 			Flashes our led at 20hz indefinitely.
 */
void HardFault_Handler()
{
	for (;;)
	{
		// Do the "hard fault panic" dance
		GPIOC_PSOR = ( 1 << 5 );
		dumbdelay_ms( 50 );
		GPIOC_PCOR = ( 1 << 5 );
		dumbdelay_ms( 50 );
	}
}

/**
 * @brief		Initialises the onboard LED.
 */
static void init_led( void )
{
	// Initialise on board LED
	PORTC_PCR5 = PORT_PCR_MUX( 0x1 );	// LED is on PC5 (pin 13), config as GPIO (alt = 1)
	GPIOC_PDDR = ( 1 << 5 );			// make this an output pin
	GPIOC_PCOR = ( 1 << 5 );			// start with LED off
}

int port_putchar( int c )
{
	uart_putchar( UART0_BASE_PTR, (char)c );
	return 1;
}

int port_getchar( void )
{
	return uart_getchar( UART0_BASE_PTR );
}

int port_getchar_nonblock( void )
{
	return uart_getchar_nonblock( UART0_BASE_PTR );
}

/**
 * @brief		Blinks a number of tixFlightTimerHandlermes with a given interval.
 * @param[in]	reps		Number of times to blink.
 * @param[in]	period_ms	Blink period (ms).
 */
static void blink( const int reps, const int period_ms )
{
	int halfperiod_ms = period_ms / 2;
	int idx;

	for ( idx = 0; idx < reps; idx ++ )
	{
		// Set LED
		GPIOC_PSOR = ( 1 << 5 );
		dumbdelay_ms( halfperiod_ms );

		// Clear LED
		GPIOC_PCOR = ( 1 << 5 );
		dumbdelay_ms( halfperiod_ms );
	}
}

/**
 * @brief		If enabled, this hook will be called in case of a stack
 * 				overflow.
 * @param[in]	pxTask		Task handle.
 * @param[in]	pcTaskName	Pointer to task name.
 */
void vApplicationStackOverflowHook( xTaskHandle pxTask, char *pcTaskName )
{
	/* This will get called if a stack overflow is detected during the context
	 switch.  Set configCHECK_FOR_STACK_OVERFLOWS to 2 to also check for stack
	 problems within nested interrupts, but only do this for debug purposes as
	 it will increase the context switch time. */
	(void)pxTask;
	(void)pcTaskName;
	taskDISABLE_INTERRUPTS();
	/* Write your code here ... */
	//for(;;) {}
	while ( 1 )
	{
		blink( 1, 100 );
	}
}

/**
 * @brief		If enabled, this hook will be called by the RTOS for every
 *				tick increment.
 */
void vApplicationTickHook( void )
{
	/* Called for every RTOS tick. */
	/* Write your code here ... */
}

/**
 * @brief		If enabled, this hook will be called when the RTOS is idle.
 *				This might be a good place to go into low power mode.
 */
void vApplicationIdleHook( void )
{
	/* Called whenever the RTOS is idle (from the IDLE task).
	 Here would be a good place to put the CPU into low power mode. */
	/* Write your code here ... */
}

/**
 * @brief		If enabled, the RTOS will call this hook in case memory
 *				allocation failed.
 */
void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	 free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	 internally by FreeRTOS API functions that create tasks, queues, software
	 timers, and semaphores.  The size of the FreeRTOS heap is set by the
	 configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	taskDISABLE_INTERRUPTS();
	/* Write your code here ... */
	while ( 1 )
	{
		blink( 1, 100 );
	}
}

/**
** @brief		Entry point to program.
** @return		Error code.
*/
int main( void )
{
	// Initialise the Teensy's on-board LED and our LED pattern controller
	init_led();

	// Initialise the UART module for comms with the ESP module
	uart_init( UART0_BASE_PTR, 115200 );

	// The IO driver handles setting up the FTM for receiver inputs and motor
	// outputs
	IODRIVER_Setup();

	// Initialise I2C which is used to talk to the LSM9DS0 IMU module
	// TODO check status?
	i2c_init( 0, 0x01, 0x20 );

	// Create tasks
	TASK_FLIGHT_Create();
	TASK_COMMS_Create();
	TASK_LED_Create();

	// Flash a little startup sequence, this isn't necessary at all, just nice
	// to see a familiar sign before things start breaking!
	blink( STARTUP_BLINK_COUNT, STARTUP_BLINK_PERIOD );

	// Say hello - printf is piped through the uart!
	printf( "Hello from TeensyQuad!\r\n" );

	// Start the tasks and timer running, this should never return as FreeRTOS
	// will branch directly into the idle task.
	vTaskStartScheduler();

	// We should never get here, this return is just to keep the compiler happy
	return 0;
}
