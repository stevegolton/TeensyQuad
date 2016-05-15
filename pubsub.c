/* ************************************************************************** **
 * Includes
 * ************************************************************************** */
#include "pubsub.h"

#include <stdint.h>			// std types
#include <stdbool.h>		// bool definition
#include <stddef.h>			// size_t

#include "FreeRTOS.h"		// FreeRTOS
#include "FreeRTOSConfig.h"	// FreeRTOS portable config
#include "portmacro.h"		// Portable functions
#include "queue.h"			// FreeRTOS queues
#include "task.h"			// taskENTER_CRITICAL()

#include "IPC_types.h"		// stFlightDetails_t

/* ************************************************************************** **
 * Macros and Defines
 * ************************************************************************** */
#define MAX_SUBS		( 16 )

/* ************************************************************************** **
 * Typedefs
 * ************************************************************************** */
struct stSubscription
{
	uint32_t uiTopic;
	size_t sMsgSize;
	QueueHandle_t xQueue;
};

/* ************************************************************************** **
 * Function Prototypes
 * ************************************************************************** */

/* ************************************************************************** **
 * Local Variables
 * ************************************************************************** */
static struct stSubscription astSubs[ MAX_SUBS ];
static size_t sNumSubs;

/* ************************************************************************** **
 * API Functions
 * ************************************************************************** */

/* ************************************************************************** */
void PUBSUB_Create( void )
{
	sNumSubs = 0;

	return;
}

/* ************************************************************************** */
hPUBSUB_Subscription_t PUBSUB_Subscribe( uint32_t uiTopic, size_t sMsgSize, size_t sQueueLen )
{
	struct stSubscription *pstSub = NULL;

	taskENTER_CRITICAL();

	if ( sNumSubs < MAX_SUBS )
	{
		pstSub = &astSubs[sNumSubs];
		sNumSubs++;
	}

	taskEXIT_CRITICAL();

	if ( NULL != pstSub )
	{
		pstSub->xQueue = xQueueCreate( sQueueLen, sMsgSize );
		pstSub->uiTopic = uiTopic;
		pstSub->sMsgSize = sMsgSize;

		return pstSub;
	}
	else
	{
		return NULL;
	}
}

/* ************************************************************************** */
void PUBSUB_Publish( uint32_t uiTopic, uint8_t *pbyMsg )
{
	uint32_t uiIndex;

	for ( uiIndex = 0; uiIndex < sNumSubs; uiIndex++ )
	{
		if ( astSubs[uiIndex].uiTopic == uiTopic )
		{
			xQueueSend( astSubs[uiIndex].xQueue, pbyMsg, 0 );
		}
	}

	return;
}

/* ************************************************************************** */
bool PUBSUB_Receive( hPUBSUB_Subscription_t hSubscription, void *pbyMsg )
{
	if ( pdTRUE == xQueueReceive( hSubscription->xQueue, pbyMsg, 0 ) )
	{
		return true;
	}
	else
	{
		return false;
	}
}

/* ************************************************************************** */
uint32_t PUBSUB_MessagesWaiting( hPUBSUB_Subscription_t hSubscription )
{
	return uxQueueMessagesWaiting( hSubscription->xQueue );
}
