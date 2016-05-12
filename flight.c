#include <stdint.h>
#include <stddef.h>

#include "flight.h"
#include "common.h"
#include "vector3f.h"

static set_rotor_spd_t _set_rotor_spd;
static get_recvr_channel_t _get_recvr_channel;
static uint16_t _receiver_max;

void flight_setup( set_rotor_spd_t set_rotor_spd,
				   get_recvr_channel_t get_recvr_channel,
				   int16_t receiver_max )
{
	// Cache local copies of callback functions
	_set_rotor_spd = set_rotor_spd;
	_get_recvr_channel = get_recvr_channel;
	_receiver_max = receiver_max;

	return;
}

void flight_process( uint16_t timestep_ms, vector3f_t accel, vector3f_t gyro )
{
	int rcvr_idx;
	uint16_t rcvr_values[NUM_RCVR_CHANNELS];
	uint16_t motor_values[NUM_MOTORS];
	int motor_idx;

	// Fetch receiver inputs
	for ( rcvr_idx = 0; rcvr_idx < NUM_RCVR_CHANNELS; rcvr_idx++ )
	{
		rcvr_values[rcvr_idx] = _get_recvr_channel( rcvr_idx );
	}

	// TODO Update the flight model

	// Update motor outputs
	for ( motor_idx = 0; motor_idx < NUM_MOTORS; motor_idx++ )
	{
		_set_rotor_spd( motor_idx, motor_values[motor_idx] );
	}

	// Remove compiler warnings
	if ( rcvr_values[0] );

	return;
}
