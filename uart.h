/*
 * File:		uart.h
 * Purpose:     Provide common ColdFire UART routines for polled serial IO
 *
 * Notes:
 * Modified to hide getchar, putchar, and char_avail routines; those now exist
 * as statics inside the uart.c code.  Modified uart_init() to take a UART
 * number (0-2), rather than a UART base pointer, and to remove the clock
 * argument; baud calcs are now based on the selected UART.
 */

#ifndef __UART_H__
#define __UART_H__

#include "common.h"

/*
 *  These routines support access to all UARTs on the Teensy 3.x (K20).
 *  To use a UART, first call UARTInit() with the UART number (0-2) and
 *  a desired baud rate; this call marks the selected UART as active.
 *  You can then call UARTWrite() to send chars to the active UART.  Use
 *  UARTAvail() to check for received chars.  Use UARTRead() to read
 *  chars (with blocking) from the active UART.
 *
 *  To use a different UART as the active UART, call UARTAssignActiveUART().
 */

/**
 * @brief		Initialises the serial port module, baud rate=115200 8N1, hw
 * 				flow control disabled.
 * @param[in]	channel		UART module's base register pointer.
 * @param[in]	baud		Baud rate in hz.
 */
void uart_init( const UART_MemMapPtr channel, const uint32_t baud );

/**
 * @brief		Get a character from the buffer.
 * @param[in]	channel		UART module's base register pointer.
 * @return		The character received from our FIFO.
 */
char uart_getchar( const UART_MemMapPtr channel );

/**
 * @brief		Get a character from the buffer - non blocking.
 * @param[in]	channel		UART module's base register pointer.
 * @return		The character received from our FIFO.
 */
int uart_getchar_nonblock( const UART_MemMapPtr channel );

/**
 * @brief		Put a character into the tx buffer.
 * @param[in]	channel		UART module's base register pointer.
 * @param[in]	ch			Character to send.
 */
void uart_putchar( const UART_MemMapPtr channel, const char c );

/**
 * @brief		Put a string into the tx buffer.
 * @param[in]	channel		UART module's base register pointer.
 * @param[in]	ch			Characters to send.
 */
void uart_puts( const UART_MemMapPtr channel, const char *const c );

#endif /* __UART_H__ */
