#include <stdio.h>			/* printf)_ & friends */
#include "common.h"			/* uC specific dfns */
#include "FreeRTOS.h"		/* FreeRTOS */
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "timers.h"
#include "i2c.h"			/* i2c ldd */
#include "uart.h"			/* uart ldd */
#include "SFE_LSM9DS0.h"	/* LSM9DS0 driver */
#include "flight.h"			/* Flight controller */
#include "vector3f.h"		/* vector3f_t */

// Define me if you want debugging, remove me for release!
//#define configASSERT( x )     if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

#define STARTUP_BLINK_COUNT		( 3 )
#define STARTUP_BLINK_PERIOD	( 100 )

#define LSM9DS0_XM				( 0x1D ) // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G				( 0x6B ) // Would be 0x6A if SDO_G is LOW

#define LED_TICK_MS				( 100UL )

// FTM divider settings
#define FTM_FC_PS_DIV_1 0
#define FTM_FC_PS_DIV_2 1
#define FTM_FC_PS_DIV_4 2
#define FTM_FC_PS_DIV_8 3
#define FTM_FC_PS_DIV_16 4
#define FTM_FC_PS_DIV_32 5
#define FTM_FC_PS_DIV_64 6
#define FTM_FC_PS_DIV_128 7

#define FTM_MODULO		36000

static stLSM9DS0_t lsm9dso_dvr;
static TimerHandle_t led_timer = NULL;
static TaskHandle_t led_task = NULL;

static uint16_t recvr_rising[6];
static uint16_t recvr_ontime[6];

/**
 * @brief		Delay using a loop. Milliseconds are very approximate based on
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

void init_i2c( void )
{
	// Enable interrupt in NVIC and set priority to 0
	// See the vector channel assignments in K20P121M100SF2RM.pdf page 69
	// The interupt for I2C0 is number 79:
	// 24 / 32 = 0
	// 24 % 32 = 24
	// Therefore we choose the 0th register and set bit 24 (INT_I2C0 - 16)
	NVICICPR0 |= ( 1 << 24 );
	NVICISER0 |= ( 1 << 24 );
	NVICIP24 = 0x00;

	SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

	PORTB_PCR2 = PORT_PCR_MUX(0x02) | PORT_PCR_ODE_MASK;
	PORTB_PCR3 = PORT_PCR_MUX(0x02) | PORT_PCR_ODE_MASK;

	// TODO check status?
	i2c_init( 0, 0x01, 0x20 );
}

int port_putchar( int c )
{
	uart_putchar( UART0_BASE_PTR, (char)c );
	return 1;
}

/**
 * @brief		Blinks a number of times with a given interval.
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
	for(;;) {}
}

/**
 * @brief		Runs recursive flight processing.
 * @param[in]	arg		Opaque pointer to our user data.
 */
static void taskhandler_flight( void *arg )
{
	vector3f_t accel;
	vector3f_t gyro;

	for ( ;; )
	{
		/* Read the latest accel and gyro values */
		LSM9DS0_readAccel( &lsm9dso_dvr );
		LSM9DS0_readGyro( &lsm9dso_dvr );

		accel.x = LSM9DS0_calcAccel( &lsm9dso_dvr, lsm9dso_dvr.ax );
		accel.y = LSM9DS0_calcAccel( &lsm9dso_dvr, lsm9dso_dvr.ay );
		accel.z = LSM9DS0_calcAccel( &lsm9dso_dvr, lsm9dso_dvr.az );

		gyro.x = LSM9DS0_calcAccel( &lsm9dso_dvr, lsm9dso_dvr.gx );
		gyro.y = LSM9DS0_calcAccel( &lsm9dso_dvr, lsm9dso_dvr.gy );
		gyro.z = LSM9DS0_calcAccel( &lsm9dso_dvr, lsm9dso_dvr.gz );

		/* Print accel and gyro to the command line * 1000 */
		printf( "Accel = %d, %d, %d\r\n",
				(int)(LSM9DS0_calcAccel( &lsm9dso_dvr, lsm9dso_dvr.ax )*1000),
				(int)(LSM9DS0_calcAccel( &lsm9dso_dvr, lsm9dso_dvr.ay )*1000),
				(int)(LSM9DS0_calcAccel( &lsm9dso_dvr, lsm9dso_dvr.az )*1000) );

		printf( "Gyro = %d, %d, %d\r\n",
				(int)(LSM9DS0_calcGyro( &lsm9dso_dvr, lsm9dso_dvr.gx )*1000),
				(int)(LSM9DS0_calcGyro( &lsm9dso_dvr, lsm9dso_dvr.gy )*1000),
				(int)(LSM9DS0_calcGyro( &lsm9dso_dvr, lsm9dso_dvr.gz )*1000) );

		printf( "Receiver input = %d\r\n", recvr_ontime[0] );

		// Process flight controller
		flight_process( 200, accel, gyro );

		// TODO: Replace with smart sleep using timer
		vTaskDelay( 500 );
	}
}

/**
 * @brief		Runs recursive comms processing.
 * @param[in]	arg		Opaque pointer to our user data.
 */
static void taskhandler_comms( void *arg )
{
	// Suspend ourselves forever!
	vTaskSuspend( NULL );
}

/**
 * @brief		Runs the LED diagnostics reporting.
 * @param[in]	arg		Opaque pointer to our user data.
 */
static void taskhandler_led( void *arg )
{
	uint16_t blink_sequence[] = { 500, 500 };
	size_t sequence_position = 0;
	size_t sequence_len = sizeof( blink_sequence ) / sizeof( blink_sequence[0] );
	uint16_t sequence_ctr = 0;

	xTimerStart( led_timer, 0 );

	for ( ; ; )
	{
		// Suspend ourselves until some nice person resumes us...
		vTaskSuspend( NULL );

		sequence_ctr += LED_TICK_MS;

		// Set the LED depending on where we are in the sequence
		if ( 0 == ( sequence_position % 2 ) )
		{
			// Set LED
			GPIOC_PSOR = ( 1 << 5 );
		}
		else
		{
			// Clear LED
			GPIOC_PCOR = ( 1 << 5 );
		}

		if ( sequence_ctr >= blink_sequence[ sequence_position ] )
		{
			sequence_ctr = 4;
			sequence_position++;

			if ( sequence_len == sequence_position )
			{
				sequence_position = 0;
			}
		}
	}
}

/**
 * @brief		Callback for our timer, controls the LED task.
 * @param[in]	xTimer		Timer handle.
 */
static void timerCallback( TimerHandle_t xTimer )
{
	// Resume the led task
	vTaskResume( led_task );
}


static void set_rotor_spd( const size_t rotor_number, const uint16_t spd )
{
	//printf( "Set rotor spd %d = %d\r\n", rotor_number, spd );
	return;
}

static uint16_t get_recvr_channel( const size_t channel_number )
{
	//printf( "Get rxr %d\r\n", channel_number );
	return 0;
}

static void write_byte( stLSM9DS0_t * stThis, uint8_t address, uint8_t subAddress, uint8_t data )
{
	//printf( "I2Cwr %d>%d\r\n", subAddress, data );
	i2c_write_byte( 0, address, subAddress, data );

	return;
}

static uint8_t read_byte( stLSM9DS0_t * stThis, uint8_t address, uint8_t subAddress )
{
	uint8_t data;

	i2c_read_byte( 0, address, subAddress, &data );

	return data;
}

static void read_bytes( stLSM9DS0_t * stThis, uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count )
{
	i2c_read_bytes( 0, address, subAddress, dest, count );

	return;
}

/**
 * @brief		Initialise the FTM module for PWM and Input Capture.
 *
 * This is a bit cheeky, this setup should be made more generalized for the
 * FTM and should be placed into a
 */
static void init_ftm0( void )
{
	/* SIM_SCGC6: FTM0=1 - Enable FTM module clock (also set this otherwise hard fault... I don't know why!) */
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;

	/* FTM0_MODE:FAULTIE=0,FAULTM=0,CAPTEST=0,PWMSYNC=0,WPDIS=1,INIT=0,FTMEN=0 */
	FTM0_MODE = (FTM_MODE_FAULTM(0x00) | FTM_MODE_WPDIS_MASK); /* Set up mode register */

	/* Turn off the module completely */
	/* FTM0_SC: TOF=0,TOIE=0,CPWMS=0,CLKS=0,PS=0 */
	FTM0_SC = (FTM_SC_CLKS(0x00) | FTM_SC_PS(0x00)); /* Clear status and control register turning the module off */
	FTM0_CNTIN = FTM_CNTIN_INIT( 0x00 );	/* Clear initial counter register */

	/* Set up FTM0's modulo register - this is the value at which it will wrap when counting */
	FTM0_MOD = FTM_MOD_MOD( FTM_MODULO );

	/* FTM0 CH0 - Configure for PWM */
	//FTM0_C0SC = ( FTM_CnSC_MSB_MASK | FTM_CnSC_ELSA_MASK );		/* Set up channel 0 status and control register to be a PWM channel, edge aligned, starts high becomes low after match */
	//FTM0_C0V = FTM_CnV_VAL( 24000 );							/* Set up channel 0 value register */

	/* FTM0 CH4 - Configure for input capture */
	FTM0_C4SC = ( FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK | FTM_CnSC_CHIE_MASK );	// Configure the channel 1 mode register by turning on the rising edge, falling edge and interrupt enable modes

	/* Set up Initial State for Channel Output register */
	FTM0_OUTINIT = FTM_OUTINIT_CH0OI_MASK;

	/* FTM0_MODE: FAULTIE=0,FAULTM=0,CAPTEST=0,PWMSYNC=0,WPDIS=1,INIT=1,FTMEN=0 */
	FTM0_MODE = (FTM_MODE_FAULTM(0x00) | FTM_MODE_WPDIS_MASK | FTM_MODE_INIT_MASK); /* Initialise the Output Channels */

	// Enable interrupt in NVIC and set priority to 0 */
	NVICICPR1 |= ( 1 << 30 );
	NVICISER1 |= ( 1 << 30 );
	NVICIP62 = 0x00;

	/* Turn FTM0 on setting up the clock divider to 128 */
	/* FTM0_SC: TOF=0,TOIE=0,CPWMS=0,CLKS=1,PS=0 */
	FTM0_SC = ( FTM_SC_CLKS( 0x01 ) | FTM_SC_PS( FTM_FC_PS_DIV_4 ) | FTM_SC_TOIE_MASK ); /* Set up status and control register */

	// Set up PTD4 (Teensy pin 6) as FTM0 output 0 (alt = 4).
	// Port mapping on page 225 of the user K20 manual
	PORTD_PCR4 = PORT_PCR_MUX( 0x4 );
}

/**
 * @brief		Called on an FTM interrupt.
 *
 * This function is referenced by name in the vector table in crt0.s and so is
 * called under any FTM0 interrupt. This may include channel interrupts and
 * overflow interrupts if they are configured. Here we check for both.
 */
void FTM0_IRQHandler( void )
{
	uint16_t now;

	// First of all we need to tell which interrupt just happened by checking
	// the appropriate interrupt flag.
	if ( FTM0_C4SC & FTM_CnSC_CHF_MASK )
	{
		// We must clear the interrupt flag here
		FTM0_C4SC &= ~FTM_CnSC_CHF_MASK;

		// Check the state of out input pin and report over serial
		if ( GPIOD_PDIR & ( 0x1 << 4 ) )
		{
			// Rising edge
			recvr_rising[0] = FTM0_C4V;
		}
		else
		{
			// Falling edge
			now = FTM0_C4V;

			if ( now < recvr_rising[0] )
			{
				now += FTM_MODULO;
			}

			recvr_ontime[0] = ( now - recvr_rising[0] );
		}
	}

	/* Check for overflow interrupt */
	if ( FTM0_SC & FTM_SC_TOF_MASK )
	{
		/* Clear mask */
		FTM0_SC &= ~FTM_SC_TOF_MASK;
	}
}

/**
** @brief		Entry point to program.
** @return		Error code.
*/
int main( void )
{
	// Initialise hardware and peripherals
	init_led();
	uart_init( UART0_BASE_PTR, 115200 );
	init_i2c();
	init_ftm0();

	// Initialise LSM driver and flight controller
	LSM9DS0_Setup( &lsm9dso_dvr, MODE_I2C, LSM9DS0_G, LSM9DS0_XM, write_byte, read_byte, read_bytes );
	flight_setup( set_rotor_spd, get_recvr_channel );

	// Flash a little startup sequence, this isn't necessary at all, just nice
	// to see a familiar sign before things start breaking!
	blink( STARTUP_BLINK_COUNT, STARTUP_BLINK_PERIOD );

	// Say hello - printf is piped through the uart!
	printf( "Hello, World!\r\n" );

	LSM9DS0_begin( &lsm9dso_dvr );

	// Create our flight task
	xTaskCreate( taskhandler_flight,			// The task's callback function
				 "Flight",						// Task name
				 500,							// Make the flight controller's stack large as we are likely to do many function calls
				 NULL,							// Parameter to pass to the callback function, we have nothhing to pass..
				 2,								// Priority, this is our only task so.. lets just use 0
				 NULL );						// We could put a pointer to a task handle here which will be filled in when the task is created

	// Create our comms task
	xTaskCreate( taskhandler_comms,				// The task's callback function
				 "Comms",						// Task name
				 configMINIMAL_STACK_SIZE,		// We can specify different stack sizes for each task? Cool!
				 NULL,							// Parameter to pass to the callback function, we have nothhing to pass..
				 1,								// Priority, this is our only task so.. lets just use 0
				 NULL );						// We could put a pointer to a task handle here which will be filled in when the task is created

	// Create our comms task
	xTaskCreate( taskhandler_led,				// The task's callback function
				 "LED_Diags",					// Task name
				 configMINIMAL_STACK_SIZE,		// We can specify different stack sizes for each task? Cool!
				 NULL,							// Parameter to pass to the callback function, we have nothhing to pass..
				 0,								// Priority, this is our only task so.. lets just use 0
				 &led_task );					// We could put a pointer to a task handle here which will be filled in when the task is created

	// Create a timer with a period of 500ms
	led_timer = xTimerCreate( "LED_Diags_Timer", 				/* A text name, purely to help debugging. */
							  ( LED_TICK_MS / portTICK_PERIOD_MS ),	/* The timer period, in this case 500ms. */
							  pdTRUE,							/* We want this to be a recurring timer so set uxAutoReload to pdTRUE. */
							  ( void * ) 0,						/* The ID is not used, so can be set to anything. */
							  timerCallback );					/* The callback function that is called when the timer expires. */

	// Start the tasks and timer running, this should never return as FreeRTOS
	// will branch directly into the idle task.
	vTaskStartScheduler();

	// We should never get here, this return is just to keep the compiler happy
	return 0;
}
