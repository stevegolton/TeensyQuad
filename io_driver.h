#ifndef RECEIVER_H
#define RECEIVER_H

#include <stdint.h>

#define RECEIVER_FTMCLK			( 48000000 )						// 24MHz
#define RECEIVER_FTMDIV			( 4 )
#define RECEIVER_FTMTICK		( RECEIVER_FTMCLK / RECEIVER_FTMDIV )
#define RECEIVER_FTM_1MS		( RECEIVER_FTMTICK / 1000 )
#define RECEIVER_FTM_2MS		( RECEIVER_FTMTICK / 500 )

#define RECEIVER_CEIL 			( RECEIVER_FTM_2MS )				// Max is 2ms
#define RECEIVER_FLOOR 			( RECEIVER_FTM_1MS )				// Min is 1ms
#define RECEIVER_CENTER			( RECEIVER_FTM_1MS + ( RECEIVER_FTM_1MS / 2 ) ) // Centre point - 1.5ms
#define RECEIVER_RANGE 			( RECEIVER_CEIL - RECEIVER_FLOOR )
#define RECEIVER_NUM_CHAN_IN	( 6 )
#define RECEIVER_NUM_CHAN_OUT	( 4 )

void IODRIVER_Setup( void );
void IODRIVER_FTM_ISR( void );
void IODRIVER_Tick( uint32_t interval_millis );
int IODRIVER_GetInputPulseWidth( int channel, uint32_t *pulseDurationTicks );
int IODRIVER_GetOutputPulseWidth( int channel, uint32_t *pulseDurationTicks );
int IODRIVER_SetOutputPulseWidth( int channel, uint32_t pulseDurationTicks );

#endif
