// telemetry_thread.c
//
// UPDATED:
// - Compute a "correct" synced time-of-day (ms since midnight UTC) using the GPS-derived
//   global offset from neom9n_thread.c
// - Use that synced time for time-sync announcing / any timestamps you want to be GPS-based
//
// Assumptions:
// - neom9n_thread.c exports:
//      volatile int64_t  g_gps_time_offset_ms;
//      volatile uint32_t g_gps_time_offset_valid;
// - That offset is computed against local ThreadX time-of-day (local_ms % 86400000).
//
// Result:
//   telemetry_gps_tod_ms() returns "GPS time-of-day in ms" when valid,
//   otherwise falls back to local ThreadX ms (mod 1 day).

#include "RF-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "can_bus.h"

#include <stdint.h>

TX_THREAD telemetry_thread;
#define TIMESYNC_ANNOUNCE_PERIOD_MS 1000u
#define TELEMETRY_THREAD_STACK_SIZE 1024u
ULONG telemetry_thread_stack[TELEMETRY_THREAD_STACK_SIZE / sizeof(ULONG)];

// Provided by neom9n_thread.c
extern volatile int64_t  g_gps_time_offset_ms;
extern volatile uint32_t g_gps_time_offset_valid;

#ifndef TX_TIMER_TICKS_PER_SECOND
#error "TX_TIMER_TICKS_PER_SECOND must be defined by ThreadX."
#endif

static uint64_t tx_raw_now_ms(void) {
    // ThreadX ticks -> ms
    ULONG ticks = tx_time_get();
    return ((uint64_t)(uint32_t)ticks * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

static uint64_t wrap_day_ms(uint64_t ms) {
    return ms % 86400000ULL;
}

// Returns GPS-synced "time-of-day" in ms since midnight UTC when valid.
// Falls back to local time-of-day if GPS is not valid yet.
static uint64_t telemetry_gps_tod_ms(void) {
    const uint64_t local_ms = tx_raw_now_ms();
    const uint64_t local_tod = wrap_day_ms(local_ms);

    if (!g_gps_time_offset_valid) {
        return local_tod;
    }

    const int64_t off = g_gps_time_offset_ms;

    // Compute local_tod + offset (signed), wrap into [0, 86400000)
    int64_t tod = (int64_t)local_tod + off;

    // Normalize wrap
    const int64_t DAY = 86400000LL;
    tod %= DAY;
    if (tod < 0) tod += DAY;

    return (uint64_t)tod;
}

void telemetry_thread_entry(ULONG initial_input)
{
    (void)initial_input;
    (void)init_telemetry_router();

    const char started_txt[] = "Telemetry thread starting";
    (void)log_telemetry_synchronous(SEDS_DT_MESSAGE_DATA,
                                    started_txt,
                                    sizeof(started_txt),
                                    1);

    uint64_t last_announce_ms = 0;

    for (;;) {
        can_bus_process_rx();
        (void)process_all_queues_timeout(5);
        can_bus_process_rx();

        // Use GPS-synced time-of-day for scheduling announce as well.
        // This keeps the announce "based off GPS time" once it becomes valid,
        // but still works before GPS lock.
        const uint64_t now_ms = telemetry_gps_tod_ms();

        if ((uint64_t)(now_ms - last_announce_ms) >= (uint64_t)TIMESYNC_ANNOUNCE_PERIOD_MS) {
            // Announce as master (priority lower = preferred).
            // If this node is NOT the master in your topology, you should NOT call this.
            (void)telemetry_timesync_announce(10ULL);
            last_announce_ms = now_ms;
        }

        tx_thread_sleep(1);
    }
}

void create_telemetry_thread(void)
{
    UINT status = tx_thread_create(&telemetry_thread,
                                   "Telemetry Thread",
                                   telemetry_thread_entry,
                                   0,
                                   telemetry_thread_stack,
                                   TELEMETRY_THREAD_STACK_SIZE,
                                   5,
                                   5,
                                   TX_NO_TIME_SLICE,
                                   TX_AUTO_START);

    if (status != TX_SUCCESS) {
        die("Failed to create telemetry thread: %u", (unsigned)status);
    }
}
