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
		0.0f
	},
	{
		"PIDGainRate_D",
		0.0f
	},
	{
		"PIDGainAngle_P",
		0.0f
	},
	{
		"PIDGainRateYaw_P",
		0.0f
	},
	{
		"PIDGainRateYaw_D",
		0.0f
	},
	{
		"TrimRoll",
		-0.014f
	},
	{
		"TrimPitch",
		0.080f
	},
	{
		"TrimYaw",
		-0.0f
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
		if ( 0 != uiLen )
		{
			if ( 0 == strncmp( sName, astParamList[sParamIndex].sName, uiLen ) )
			{
				break;
			}
		}
		else
		{
			if ( 0 == strcmp( sName, astParamList[sParamIndex].sName ) )
			{
				break;
			}
		}
	}

	if ( sParamIndex != mArrayLen( astParamList ) )
	{
		if ( puiIndex )
		{
			*puiIndex = sParamIndex;
		}
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

