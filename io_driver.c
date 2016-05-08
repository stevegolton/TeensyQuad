/**
 * io_driver
 * @brief	Singleton module - handles all your PWM related IO!
 *
 * Currently, the FTM tick is 0.167us (24MHz/4) with a modulo of 36000 to give
 * a period of 3ms.
 *
 * TODO Fully configure all FTM modules from here to ensure consistency.
 */
#include <io_driver.h>	// Module header file
#include <stdlib.h>
#include <stdint.h>

#include "MK20D7.h"		// Chip definitions for FTM registers

// TODO Perhaps we could make these configurable in a construction function?
#define TIMEOUT_MILLIS	( 500 )
#define FTM_MODULO		( 36000 )
#define FTM_DIV			( 4 )

// FTM things
#define FTM_FC_PS_DIV_1 0
#define FTM_FC_PS_DIV_2 1
#define FTM_FC_PS_DIV_4 2
#define FTM_FC_PS_DIV_8 3
#define FTM_FC_PS_DIV_16 4
#define FTM_FC_PS_DIV_32 5
#define FTM_FC_PS_DIV_64 6
#define FTM_FC_PS_DIV_128 7

#define PERIOD_1MS ( 12000 )
#define PERIOD_2MS ( 2 * PERIOD_1MS )
#define PERIOD_3MS ( 3 * PERIOD_1MS )

typedef struct
{
	FTM_MemMapPtr ftmBasePtr;
	uint32_t chan;
	GPIO_MemMapPtr porttr;
	uint8_t shift;
	uint32_t timeStampOfRisingEdge;
	uint32_t pulseDuration;
	uint32_t timeout_millis;

} chanInCtx_t;

typedef struct
{
	FTM_MemMapPtr ftmBasePtr;
	uint32_t chan;
	GPIO_MemMapPtr porttr;
	uint8_t shift;
	uint32_t pulseDuration;

} chanOutCtx_t;

/**
 * Maps receiver channel numbers to ports and FTM channels for easy access.
 */
static chanInCtx_t chanInCtxList[] =
{
	{ FTM0_BASE_PTR, 4, PTD_BASE_PTR,  4, 0, 0, 0 },
	{ FTM0_BASE_PTR, 5, PTD_BASE_PTR,  5, 0, 0, 0 },
	{ FTM0_BASE_PTR, 6, PTD_BASE_PTR,  6, 0, 0, 0 },
	{ FTM0_BASE_PTR, 7, PTD_BASE_PTR,  7, 0, 0, 0 },
	{ FTM1_BASE_PTR, 0, PTA_BASE_PTR, 12, 0, 0, 0 },
	{ FTM1_BASE_PTR, 1, PTA_BASE_PTR, 13, 0, 0, 0 },
};

static chanOutCtx_t chanOutCtxList[] =
{
	{ FTM0_BASE_PTR, 0, PTC_BASE_PTR,  1, 0 },
	{ FTM0_BASE_PTR, 1, PTC_BASE_PTR,  2, 0 },
	{ FTM0_BASE_PTR, 2, PTC_BASE_PTR,  3, 0 },
	{ FTM0_BASE_PTR, 3, PTC_BASE_PTR,  4, 0 },
};

static void initFTM0( void );
static void initFTM1( void );

/**
 * @brief		Sets up all the FTM bits a bobs.
 */
void IODRIVER_Setup( void )
{
	initFTM0();
	initFTM1();
}

/**
 * @brief		Gets the latest pulse duration for a given channel.
 *
 * @param[in]	channel		The ID of the channel to read, must be in the range
 * 							0..RECEIVER_NUM_CHAN_IN.
 * @param[out]	pulseDurationTicks The value in ticks.
 *
 * @returns		Error code, success if != 0.
 */
int IODRIVER_GetInputPulseWidth( int channel, uint32_t *pulseDurationTicks )
{
	uint32_t pulseDuration;

	if ( channel < RECEIVER_NUM_CHAN_IN )
	{
		// We don't have to turn off interrupts when accessing this as 32-bit
		// numbers are an atomic operation on the K20.
		pulseDuration = chanInCtxList[channel].pulseDuration;

		// FIXME should this be limited here or later down the pipeline?
		if (    ( RECEIVER_CEIL < pulseDuration  )
			 || ( RECEIVER_FLOOR > pulseDuration ) )
		{
			// Reading is out of bounds... bummer!
			pulseDuration = RECEIVER_FLOOR;
		}

		*pulseDurationTicks = pulseDuration;

		return 1;
	}
	else
	{
		return 0;
	}
}

/**
 * @brief		Gets the latest pulse duration for a given channel.
 *
 * @param[in]	channel		The ID of the channel to read, must be in the range
 * 							0..RECEIVER_NUM_CHAN_OUT.
 * @param[out]	pulseDurationTicks The value in ticks.
 *
 * @returns		Error code, success if != 0.
 */
int IODRIVER_GetOutputPulseWidth( int channel, uint32_t *pulseDurationTicks )
{
	uint32_t pulseDuration;

	if ( channel < RECEIVER_NUM_CHAN_OUT )
	{
		*pulseDurationTicks = chanOutCtxList[channel].pulseDuration;

		return 1;
	}
	else
	{
		return 0;
	}
}

/**
 * @brief		Sets the pulse duration of one of the output channels.
 *
 * @param[in]	channel		The ID of the channel to set, must be in the range
 * 							0..RECEIVER_NUM_CHAN_OUT.
 * @param[in]	pulseDurationTicks The value in ticks.
 *
 * @returns		Error code, success if != 0.
 */
int IODRIVER_SetOutputPulseWidth( int channel, uint32_t pulseDurationTicks )
{
	chanOutCtx_t *ctx;

	if ( channel < RECEIVER_NUM_CHAN_OUT )
	{
		ctx = &chanOutCtxList[channel];

		ctx->pulseDuration = pulseDurationTicks;

		return 1;
	}
	else
	{
		return 0;
	}
}

/**
 * @brief		ISR handler for FTM 0 and 1.
 */
void FTM0_IRQHandler( void )
{
	int chanindex;
	chanInCtx_t* inCtx;
	chanOutCtx_t *outCtx;
	int pinvalue;
	uint32_t timeStampNow;

	for ( chanindex = 0; chanindex < RECEIVER_NUM_CHAN_IN; chanindex++ )
	{
		inCtx = &chanInCtxList[chanindex];

		// Check the channel's interrupt flag
		if ( ( FTM_CnSC_REG( inCtx->ftmBasePtr, inCtx->chan ) & 0x80 ) != 0 )
		{
			// Clear channel interrupt flag
			FTM_CnSC_REG( inCtx->ftmBasePtr, inCtx->chan ) &= ~0x80;

			// Flag is set, check the pin value to see if it's a rising or a
			// falling edge
			pinvalue = ( GPIO_PDIR_REG( inCtx->porttr ) >> inCtx->shift ) & 0x1;

			if (  0 != pinvalue )
			{
				// Rising edge, record its time
				inCtx->timeStampOfRisingEdge = FTM_CnV_REG( inCtx->ftmBasePtr, inCtx->chan );
			}
			else
			{
				// Falling edge, calculate the duration of the pulse
				timeStampNow = FTM_CnV_REG( inCtx->ftmBasePtr, inCtx->chan );

				if ( timeStampNow < inCtx->timeStampOfRisingEdge )
				{
					timeStampNow += FTM_MODULO;
				}

				inCtx->pulseDuration = ( timeStampNow - inCtx->timeStampOfRisingEdge );

				// Kick the timeout watchdog
				inCtx->timeout_millis = 0;
			}
		}
	}

	// Check for FTM overflow and move output pulse values into FTM registers
	// this synchronises the output pulse assignment in order to avoid little
	// ghost pulses as an artifact of the pulse value changing by large amounts
	if ( ( FTM0_SC & 0x80 ) != 0 )
	{
		// Clear interrupt flag
		FTM0_SC &= ~0x80;

		for ( chanindex = 0; chanindex < RECEIVER_NUM_CHAN_OUT; chanindex++ )
		{
			outCtx = &chanOutCtxList[chanindex];

			// Assign the value to the FTM channel value register
			FTM_CnV_REG( outCtx->ftmBasePtr, outCtx->chan ) = FTM_CnV_VAL( outCtx->pulseDuration );
		}

		// Check and update "watchdog" timeouts
		for ( chanindex = 0; chanindex < RECEIVER_NUM_CHAN_IN; chanindex++ )
		{
			chanInCtxList[chanindex].timeout_millis += 3;

			if ( chanInCtxList[chanindex].timeout_millis > TIMEOUT_MILLIS )
			{
				chanInCtxList[chanindex].pulseDuration = RECEIVER_FLOOR;
			}
		}
	}

	if ( ( FTM1_SC & 0x80 ) != 0 )
	{
		// Clear interrupt flag
		FTM1_SC &= ~0x80;
	}

	return;
}

void FTM1_IRQHandler( void )
{
	// Pipe interrupts to the same handler...
	FTM0_IRQHandler();
}

/**
 * @brief		Initialise the FTM module for PWM and Input Capture.
 *
 * This is a bit cheeky, this setup should be made more generalised.
 */
static void initFTM0( void )
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
	FTM0_C5SC = ( FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK | FTM_CnSC_CHIE_MASK );	// Configure the channel 1 mode register by turning on the rising edge, falling edge and interrupt enable modes
	FTM0_C6SC = ( FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK | FTM_CnSC_CHIE_MASK );	// Configure the channel 1 mode register by turning on the rising edge, falling edge and interrupt enable modes
	FTM0_C7SC = ( FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK | FTM_CnSC_CHIE_MASK );	// Configure the channel 1 mode register by turning on the rising edge, falling edge and interrupt enable modes

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
	PORTD_PCR5 = PORT_PCR_MUX( 0x4 );
	PORTD_PCR6 = PORT_PCR_MUX( 0x4 );
	PORTD_PCR7 = PORT_PCR_MUX( 0x4 );

	return;
}

/**
 * @brief		Initialise the FTM module for PWM and Input Capture.
 *
 * This is a bit cheeky, this setup should be made more generalised.
 */
static void initFTM1( void )
{
	/* SIM_SCGC6: FTM0=1 - Enable FTM module clock (also set this otherwise hard fault... I don't know why!) */
	SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;

	/* FTM0_MODE:FAULTIE=0,FAULTM=0,CAPTEST=0,PWMSYNC=0,WPDIS=1,INIT=0,FTMEN=0 */
	FTM1_MODE = (FTM_MODE_FAULTM(0x00) | FTM_MODE_WPDIS_MASK); /* Set up mode register */

	/* Turn off the module completely */
	/* FTM0_SC: TOF=0,TOIE=0,CPWMS=0,CLKS=0,PS=0 */
	FTM1_SC = (FTM_SC_CLKS(0x00) | FTM_SC_PS(0x00)); /* Clear status and control register turning the module off */
	FTM1_CNTIN = FTM_CNTIN_INIT( 0x00 );	/* Clear initial counter register */

	/* Set up FTM0's modulo register - this is the value at which it will wrap when counting */
	FTM1_MOD = FTM_MOD_MOD( FTM_MODULO );

	/* FTM0 CH4 - Configure for input capture */
	FTM1_C0SC = ( FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK | FTM_CnSC_CHIE_MASK );	// Configure the channel 1 mode register by turning on the rising edge, falling edge and interrupt enable modes
	FTM1_C1SC = ( FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK | FTM_CnSC_CHIE_MASK );	// Configure the channel 1 mode register by turning on the rising edge, falling edge and interrupt enable modes

	/* Set up Initial State for Channel Output register */
	FTM1_OUTINIT = FTM_OUTINIT_CH0OI_MASK;

	/* FTM1_MODE: FAULTIE=0,FAULTM=0,CAPTEST=0,PWMSYNC=0,WPDIS=1,INIT=1,FTMEN=0 */
	FTM1_MODE = (FTM_MODE_FAULTM(0x00) | FTM_MODE_WPDIS_MASK | FTM_MODE_INIT_MASK); /* Initialise the Output Channels */

	// Enable interrupt in NVIC and set priority to 0 */
	NVICICPR1 |= ( 1 << 31 );
	NVICISER1 |= ( 1 << 31 );
	NVICIP63 = 0x00;

	/* Turn FTM1 on setting up the clock divider to 128 */
	/* FTM1_SC: TOF=0,TOIE=0,CPWMS=0,CLKS=1,PS=0 */
	FTM1_SC = ( FTM_SC_CLKS( 0x01 ) | FTM_SC_PS( FTM_FC_PS_DIV_4 ) | FTM_SC_TOIE_MASK ); /* Set up status and control register */

	// Set up port mappings
	// Port mapping on page ~225 of the user K20 manual
	PORTA_PCR12 = PORT_PCR_MUX( 0x3 );
	PORTA_PCR13 = PORT_PCR_MUX( 0x3 );

	return;
}
