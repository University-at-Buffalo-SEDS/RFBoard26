// telemetry_thread.c
#include "RF-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "can_bus.h"
#include "main.h"

TX_THREAD telemetry_thread;
#define TELEMETRY_THREAD_STACK_SIZE (4U *1024U)

// How often this node requests a resync from the master:
#define TIMESYNC_REQUEST_PERIOD_MS 2000u   // e.g. every 2 seconds
// How often grand-master announces Unix time:
#define TIMESYNC_ANNOUNCE_PERIOD_MS 1000u

#ifndef TX_TIMER_TICKS_PER_SECOND
#error "TX_TIMER_TICKS_PER_SECOND must be defined by ThreadX."
#endif

static uint64_t tx_now_ms(void) {
    ULONG ticks = tx_time_get();
    return ((uint64_t)(uint32_t)ticks * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

void telemetry_thread_entry(ULONG initial_input)
{
    (void)initial_input;

    // Ensure router exists early (so we can send requests immediately)
    (void)init_telemetry_router();

    // const char started_txt[] = "Telemetry thread starting";
    // (void)log_telemetry_synchronous(SEDS_DT_MESSAGE_DATA,
    //                                 started_txt,
    //                                 sizeof(started_txt),
    //                                 1);

    uint64_t last_req_ms = 0;
    uint64_t last_announce_ms = 0;

    for (;;) {
        can_bus_poll();
        can_bus_process_rx();
        SedsResult res = process_all_queues_timeout(50);
        if (res != SEDS_OK)
        {
            printf("Telemetry thread: process_all_queues_timeout error %d\n", (int)res);
        }

        const uint64_t now_ms = tx_now_ms();
#if TELEMETRY_TIME_MASTER
        if ((uint64_t)(now_ms - last_announce_ms) >= (uint64_t)TIMESYNC_ANNOUNCE_PERIOD_MS) {
            const uint64_t unix_ms = telemetry_unix_ms();
            if (unix_ms != 0ULL) {
                (void)telemetry_timesync_announce(/*priority=*/1ULL, unix_ms);
            }
            last_announce_ms = now_ms;
        }
#else
        if ((uint64_t)(now_ms - last_req_ms) >= (uint64_t)TIMESYNC_REQUEST_PERIOD_MS) {
            (void)telemetry_timesync_request();
            last_req_ms = now_ms;
        }
#endif
        HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
        tx_thread_sleep(100);
    }
}

UINT create_telemetry_thread(TX_BYTE_POOL *byte_pool)
{

        CHAR *pointer;

  /* Allocate the stack for test  */
  if (tx_byte_allocate(byte_pool, (VOID**) &pointer,
                       TELEMETRY_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

    UINT status = tx_thread_create(&telemetry_thread,
                                   "Telemetry Thread",
                                   telemetry_thread_entry,
                                   0,
                                   pointer,
                                   TELEMETRY_THREAD_STACK_SIZE,
                                   5,
                                   5,
                                   TX_NO_TIME_SLICE,
                                   TX_AUTO_START);

    return status;
}
