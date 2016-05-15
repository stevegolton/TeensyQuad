#ifndef PUBSUB_H
#define PUBSUB_H

#include <stdint.h>			// std types
#include <stdbool.h>		// bool definition
#include <stddef.h>			// size_t

struct stSubscription;

typedef struct stSubscription* hPUBSUB_Subscription_t;

/**
 * @brief		Initialises the pubsub module.
 */
void PUBSUB_Create( void );

hPUBSUB_Subscription_t PUBSUB_Subscribe( const uint32_t uiTopic, const size_t sMsgSize, const size_t sQueueLen );
void PUBSUB_Publish( const uint32_t uiTopic, const void *const pvMsg  );
bool PUBSUB_Receive( hPUBSUB_Subscription_t hSubscription, void *const pbyMsg );
uint32_t PUBSUB_MessagesWaiting( hPUBSUB_Subscription_t hSubscription );

#endif
