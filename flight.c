#include <stdint.h>
#include <stddef.h>

#include "flight.h"
#include "common.h"

static set_rotor_spd_t _set_rotor_spd;
static get_recvr_channel_t _get_recvr_channel;

void flight_setup( set_rotor_spd_t set_rotor_spd, get_recvr_channel_t get_recvr_channel )
{
	// Cache local copies of callback functions
	_set_rotor_spd = set_rotor_spd;
	_get_recvr_channel = get_recvr_channel;

	return;
}

void flight_process( void )
{
	int rcvr_idx;
	uint16_t rcvr_values[NUM_RCVR_CHANNELS];
	int motor_idx;

	for ( rcvr_idx = 0; rcvr_idx < NUM_RCVR_CHANNELS; rcvr_idx++ )
	{
		rcvr_values[rcvr_idx] = _get_recvr_channel( rcvr_idx );
	}

	for ( motor_idx = 0; motor_idx < NUM_MOTORS; motor_idx++ )
	{
		_set_rotor_spd( motor_idx, 0 );
	}

	return;
}
