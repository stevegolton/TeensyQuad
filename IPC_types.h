#ifndef IPC_TYPES_H
#define IPC_TYPES_H

#include "vector3f.h"		// vector3f_t

#define LEN_PATTERN_MAX		( 16 )

#define TOPIC_FLIGHT_DETAILS	( 0 )
#define TOPIC_LED_PATTERN		( 1 )

typedef struct
{
	vector3f_t stAttitude;
	vector3f_t stAttitudeRate;
	uint16_t uiFlightRunCount;

} stFlightDetails_t;

typedef struct
{
	uint16_t auiPattern[ LEN_PATTERN_MAX ];
	size_t sPatternLen;

} stLedPattern_t;

#endif
