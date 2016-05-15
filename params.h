#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>			// std types
#include <stdbool.h>		// bool definition
#include <stddef.h>			// size_t

#define LEN_NAME_MAX		( 16 )

typedef struct
{
	char sName[ LEN_NAME_MAX ];
	float fValue;

} stPARAM_t;

size_t PARAM_GetParamCount( void );
stPARAM_t *PARAM_GetParamList( void );
stPARAM_t *PARAM_FindParamByName( const char* const sName, const size_t uiLen, size_t *puiIndex );

#endif
