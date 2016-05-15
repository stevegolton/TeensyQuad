/* ************************************************************************** **
 * Includes
 * ************************************************************************** */
#include "params.h"

#include <stdint.h>			// std types
#include <stdbool.h>		// bool definition
#include <stddef.h>			// size_t
#include <string.h>			// memset & friends
#include <stdio.h>			// printf & friends

/* ************************************************************************** **
 * Macros and Defines
 * ************************************************************************** */
#define mArrayLen( x )		( sizeof( x ) / sizeof( x[0] ) )

/* ************************************************************************** **
 * Typedefs
 * ************************************************************************** */

/* ************************************************************************** **
 * Function Prototypes
 * ************************************************************************** */

/* ************************************************************************** **
 * Local Variables
 * ************************************************************************** */
static stPARAM_t astParamList[] =
{
	{
		"PIDGainRate_P",
		0.0F
	},
	{
		"PIDGainRate_D",
		0.0F
	},
	{
		"PIDGainAngle_P",
		0.0F
	},
	{
		"PIDGainRateYaw_P",
		0.0F
	},
	{
		"PIDGainRateYaw_D",
		0.0F
	}
};

/* ************************************************************************** **
 * API Functions
 * ************************************************************************** */

/* ************************************************************************** */
size_t PARAM_GetParamCount( void )
{
	return mArrayLen( astParamList );
}

/* ************************************************************************** */
stPARAM_t *PARAM_GetParamList( void )
{
	return &astParamList[0];
}

/* ************************************************************************** */
stPARAM_t *PARAM_FindParamByName( const char* const sName, const size_t uiLen, size_t *puiIndex )
{
	size_t sParamIndex;

	for ( sParamIndex = 0; sParamIndex < mArrayLen( astParamList ); sParamIndex++ )
	{
		if ( 0 == strncmp( sName, astParamList[sParamIndex].sName, uiLen ) )
		{
			break;
		}
	}

	if ( sParamIndex != mArrayLen( astParamList ) )
	{
		*puiIndex = sParamIndex;
		return &astParamList[sParamIndex];
	}
	else
	{
		return NULL;
	}
}

/* ************************************************************************** **
 * Local Functions
 * ************************************************************************** */

