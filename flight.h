#ifndef FLIGHT_H
#define FLIGHT_H

#include <stdint.h>
#include <stddef.h>

#define NUM_MOTORS			( 4 )
#define NUM_RCVR_CHANNELS	( 6 )

typedef void (*set_rotor_spd_t)( const size_t rotor_number, const uint16_t spd );
typedef uint16_t (*get_recvr_channel_t)( const size_t channel_number );

/**
 * @brief		Initialise the flight controller.
 */
void flight_setup( set_rotor_spd_t set_rotor_spd,
				   get_recvr_channel_t get_recvr_channel );

/**
 * @brief		Update the flight controller model.
 */
void flight_process( uint16_t timestep_ms, uint16_t *accel, uint16_t *gyro );

#endif
