#ifndef FLIGHT_H
#define FLIGHT_H

#include <stdint.h>
#include <stddef.h>

#define NUM_MOTORS			( 4 )
#define NUM_RCVR_CHANNELS	( 6 )

typedef void (*set_rotor_spd_t)( const size_t rotor_number, const uint16_t spd );
typedef uint16_t (*get_recvr_channel_t)( const size_t channel_number );

void flight_setup( set_rotor_spd_t set_rotor_spd, get_recvr_channel_t get_recvr_channel );
void flight_process( void );

#endif
