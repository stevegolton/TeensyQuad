#ifndef FLIGHT_H
#define FLIGHT_H

#include <stdint.h>
#include <stddef.h>
#include "vector3f.h"

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
 * @param[in]	timestep_ms	The timestep since the last time we were called.
 * @param[in]	accel		Array of 3 pointers to accelerometer values 0.001g/bit.
 * @param[in]	gyro		Array of 3 pointers to gyro values 0.001rad/s/bit.
 */
void flight_process( uint16_t timestep_ms, vector3f_t accel, vector3f_t gyro );

#endif
