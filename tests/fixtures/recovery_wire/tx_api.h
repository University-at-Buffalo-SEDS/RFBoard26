
#pragma once
#include <stdint.h>
typedef unsigned UINT;typedef unsigned long ULONG;typedef void VOID;
typedef struct{UINT locked;}TX_MUTEX;typedef struct{ULONG count;}TX_SEMAPHORE;typedef struct{int unused;}TX_BYTE_POOL;
#define TX_NULL ((void*)0)
#define TX_SUCCESS 0U
#define TX_NO_INSTANCE 1U
#define TX_MUTEX_ERROR 2U
#define TX_QUEUE_ERROR 3U
#define TX_POOL_ERROR 4U
#define TX_NO_WAIT 0U
#define TX_WAIT_FOREVER 0xffffffffUL
#define TX_INHERIT 1U
UINT tx_mutex_create(TX_MUTEX*,const char*,UINT);UINT tx_mutex_get(TX_MUTEX*,ULONG);UINT tx_mutex_put(TX_MUTEX*);
UINT tx_semaphore_create(TX_SEMAPHORE*,const char*,ULONG);UINT tx_semaphore_get(TX_SEMAPHORE*,ULONG);UINT tx_semaphore_put(TX_SEMAPHORE*);

#define TX_CALLER_ERROR 17U
void *tx_thread_identify(void);

#define TX_TIMER_TICKS_PER_SECOND 1000U
ULONG tx_time_get(void);

#ifndef TX_TIMER_TICKS_PER_SECOND
#define TX_TIMER_TICKS_PER_SECOND 1000
#endif
ULONG tx_time_get(void);

#define TX_NOT_AVAILABLE ((UINT)0x1d)
#define TX_WAIT_ABORTED ((UINT)0x1a)
