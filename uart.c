/*
 * uart.c
 *
 *  Created on: 26 Apr 2016
 *      Author: steve
 */

#include "uart.h"
#include "common.h"
#include <string.h>

#define LEN_RX_BUF		( 1024 )

char achReceiveBuffer[ LEN_RX_BUF ];
size_t uiHead;
size_t uiTail;

void uart_init( const UART_MemMapPtr channel, const uint32_t baud )
{
	register uint16_t ubd, brfa;
	uint8_t temp;

	uiHead = 0;
	uiTail = 0;

	// Initialise serial port pins
	PORTB_PCR16 = PORT_PCR_MUX( 0x3 );
	PORTB_PCR17 = PORT_PCR_MUX( 0x3 );

	/* Enable the clock to UART0 */
	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;

	/* Make sure that the transmitter and receiver are disabled while we
	* change settings.
	*/
	UART_C2_REG( channel ) &= ~( UART_C2_TE_MASK | UART_C2_RE_MASK );

	/* Configure the UART for 8-bit mode, no parity */
	/* We need all default settings, so entire register is cleared */
	UART_C1_REG( channel ) = 0;

	/* Calculate baud settings */
	ubd = (uint16_t)( ( mcg_clk_khz * 1000)/( baud * 16 ) );

	/* Save off the current value of the UARTx_BDH except for the SBR */
	temp = UART_BDH_REG( channel ) & ~( UART_BDH_SBR( 0x1F ) );
	UART_BDH_REG( channel ) = temp | UART_BDH_SBR( ( ( ubd & 0x1F00 ) >> 8 ) );
	UART_BDL_REG( channel ) = (uint8_t)( ubd & UART_BDL_SBR_MASK );

	/* Determine if a fractional divider is needed to get closer to the baud rate */
	brfa = ( ( ( mcg_clk_khz * 32000 ) / ( baud * 16 ) ) - ( ubd * 32 ) );

	/* Save off the current value of the UARTx_C4 register except for the BRFA */
	temp = UART_C4_REG( channel ) & ~( UART_C4_BRFA( 0x1F ) );
	UART_C4_REG( channel ) = temp | UART_C4_BRFA( brfa );

	/* Enable receiver and transmitter */
	UART_C2_REG( channel ) |= ( UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK );

	// Enable interrupt in NVIC and set priority to 0 */
	NVICICPR1 |= ( 1 << 13 );
	NVICISER1 |= ( 1 << 13 );
	NVICIP45 = 0x00;
}

void UART0_RX_TX_IRQHandler( void )
{
	while ( UART_S1_REG( UART0_BASE_PTR ) & UART_S1_RDRF_MASK )
	{
		achReceiveBuffer[uiHead++] = UART_D_REG( UART0_BASE_PTR );
		uiHead %= LEN_RX_BUF;

		// Move the tail on if we have caught up with it
		if ( uiHead == uiTail )
		{
			uiTail++;
			uiTail %= LEN_RX_BUF;
		}
	}
}

char uart_getchar( const UART_MemMapPtr channel )
{
	/* Wait until character has been received */
	while (!(UART_S1_REG(channel) & UART_S1_RDRF_MASK));

	/* Return the 8-bit data from the receiver */
	return UART_D_REG(channel);
}

int uart_getchar_nonblock( const UART_MemMapPtr channel )
{
	char cRetVal;

	if ( uiHead == uiTail )
	{
		return -1;
	}
	else
	{
		DisableInterrupts;

		cRetVal = achReceiveBuffer[ uiTail++ ];
		uiTail %= LEN_RX_BUF;

		EnableInterrupts;

		return cRetVal;
	}

#if 0
	/* Wait until character has been received */
	if (!(UART_S1_REG(channel) & UART_S1_RDRF_MASK))
	{
		return -1;
	}
	else
	{
		/* Return the 8-bit data from the receiver */
		return UART_D_REG(channel);
	}
#endif
}

void uart_putchar( const UART_MemMapPtr channel, const char ch )
{
	/* Wait until space is available in the FIFO */
	while(!(UART_S1_REG(channel) & UART_S1_TDRE_MASK));

	/* Send the character */
	UART_D_REG(channel) = (uint8_t)ch;
}

void uart_puts( UART_MemMapPtr channel, const char *const s )
{
	int i;

	for ( i = 0; i < strlen( s ); i++ )
	{
		uart_putchar( channel, s[i] );
	}
}
