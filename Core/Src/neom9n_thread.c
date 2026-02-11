// neom9n_thread.c
//
// UPDATED:
// - Maintains a globally-accessible GPS-synced offset (ms) that other threads
// (telemetry thread)
//   can read to convert local ThreadX-relative time into GPS time-of-day.
// - Offset definition:
//      gps_time_of_day_ms ≈ (local_tx_now_ms % 86400000) + g_gps_time_offset_ms
//
// Notes:
// - This is "time-of-day" sync (seconds since midnight UTC), because your
// current GPS time packet
//   appears to be "seconds since midnight". If you later expose full date
//   (Y/M/D) or Unix epoch, we can make this a true epoch offset.
//
// How telemetry thread should use it:
//   extern volatile int64_t g_gps_time_offset_ms;
//   extern volatile uint32_t g_gps_time_offset_valid;
//   if (g_gps_time_offset_valid) {
//       uint64_t local_ms = tx_now_ms();
//       uint64_t gps_tod_ms = (local_ms % 86400000ULL) +
//       (int64_t)g_gps_time_offset_ms;
//   }

#include "RF-Threads.h"
#include "neom9n.h"
#include "neom9n_config.h"
#include "telemetry.h"
#include "tx_api.h"

#include <stdint.h>
#include <string.h>

// External SPI handle
extern SPI_HandleTypeDef hspi1;

// Global GPS time offset (exported symbols)
volatile int64_t g_gps_time_offset_ms = 0;     // ms
volatile uint32_t g_gps_time_offset_valid = 0; // 0/1

// Optional: expose a tiny accessor API (also globally link-visible)
int64_t gps_time_offset_ms_get(void) { return g_gps_time_offset_ms; }
uint32_t gps_time_offset_valid_get(void) { return g_gps_time_offset_valid; }

#ifndef TX_TIMER_TICKS_PER_SECOND
#error "TX_TIMER_TICKS_PER_SECOND must be defined by ThreadX."
#endif

static uint64_t tx_now_ms(void) {
    // ThreadX tick -> ms, using integer math.
    // If your tick rate does not divide 1000 evenly, this will truncate.
    // (Still fine for offset sync; if you want better precision, accumulate
    // remainder.)
    ULONG ticks = tx_time_get();
    return ((uint64_t)(uint32_t)ticks * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

static uint64_t wrap_day_ms(uint64_t ms) {
    const uint64_t DAY_MS = 86400000ULL;
    return ms % DAY_MS;
}

// Smooth offset updates to avoid jitter
#ifndef GPS_OFFSET_SMOOTH_DIV
#define GPS_OFFSET_SMOOTH_DIV 8
#endif

#ifndef GPS_OFFSET_MAX_STEP_MS
#define GPS_OFFSET_MAX_STEP_MS 60000 // ignore insane jumps > 60s
#endif

static void gps_offset_update_ms(int64_t measured_offset_ms) {
    if (measured_offset_ms > GPS_OFFSET_MAX_STEP_MS || measured_offset_ms < -GPS_OFFSET_MAX_STEP_MS) {
        return;
    }

    // Smooth: new = old + measured/Div
    int64_t step = measured_offset_ms / (int64_t)GPS_OFFSET_SMOOTH_DIV;
    if (step == 0) {
        if (measured_offset_ms > 0) {
            step = 1;
        }
        else if (measured_offset_ms < 0) {
            step = -1;
        }
    }

    g_gps_time_offset_ms += step;
    g_gps_time_offset_valid = 1;
}

// Stack + TCB for neom9n thread
TX_THREAD neom9n_thread;
#define NEOM9N_THREAD_STACK_SIZE 1024u
ULONG neom9n_thread_stack[NEOM9N_THREAD_STACK_SIZE / sizeof(ULONG)];

void neom9n_thread_entry(ULONG initial_input) {
    (void)initial_input;

    ///**
    tx_thread_sleep(2000);  // Wait 2 seconds for GPS to boot (recommended datasheet)

    // Configure for rocket flight
    // @note: Check with datasheet, after first config this block is likely reducdent 
    if (config_gps_seds_rocket(&hspi1, 5000)) {
        const char success[] = "NEOM9N config set successful";
        log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA, 
                                    success, 
                                    1, 
                                    sizeof(success)); //Log config success
    } else {
        // Factory settings are fine, so no need to terminate if config fails
        // Config method greatly reduces overhead, but failure is not critical
        // Data rate unaffected by config success or failure 
        const char failure[] = "NEOM9N config set failed";  
        log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA, 
                                    failure, 
                                    1, 
                                    sizeof(failure)); //Log config failure
    }

    tx_thread_sleep(1000); // Wait for config to settle
    //*/

    const char started_txt[] = "NEOM9N thread starting";
    log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA,
                                started_txt, 
                                1,
                                sizeof(started_txt)); // initial log statement

    // Initialize GPS packet
    NEOM9N_t gps_packet;
    gps_init(&gps_packet, &hspi1);

    const char init_txt[] = "NEOM9N packet initialized";
    log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA, 
                                init_txt, 
                                1,
                                sizeof(init_txt));

    // Binary buffers for telemetry
    uint8_t gps_data_buffer[12]; // 3 floats (lat, lon, alt)

    // Time in milliseconds since UNIX EPOCH
    uint64_t gps_time_of_day_ms = 0;

    // Error tracking
    uint32_t consecutive_errors = 0;
    uint32_t no_fix_counter = 0;

    for (;;) {
        NEOM9N_status_t status = receive_nmea(&gps_packet, NMEA_MAX_WAIT, NMEA_MAX_IGNORES);

        if (status == NEOM9N_OK) {
            consecutive_errors = 0;

            if (gps_has_fix(&gps_packet)) {
                no_fix_counter = 0;

                // Pack and send GPS position data (lat, lon, alt)
                pack_gps_data(&gps_packet, gps_data_buffer);
                log_telemetry_asynchronous(SEDS_DT_GPS_DATA, 
                                            gps_data_buffer, 
                                            3,
                                            sizeof(float));

                // Pack GPS time-of-day.
                // IMPORTANT: pack_time_data() must fill gps_time_of_day_ms as *ms since
                // midnight UTC* (or you should change it accordingly).

                // Pack GPS time-of-day (ms since UNIX Epoch) from the parsed fields.
                gps_time_of_day_ms = get_datetime_data(&gps_packet);

                // Update global offset so other threads can compute GPS time-of-day
                // from local ticks. offset := gps_tod_ms - (local_tod_ms)
                const uint64_t local_ms = tx_now_ms();
                const uint64_t local_tod = wrap_day_ms(local_ms);

                // Compute signed difference (gps - local)
                int64_t measured_offset = (int64_t)gps_time_of_day_ms - (int64_t)local_tod;

                // Handle wrap-around near midnight: choose the shortest signed delta in
                // [-12h, +12h]
                const int64_t HALF_DAY = (int64_t)(86400000LL / 2LL);
                if (measured_offset > HALF_DAY) {
                    measured_offset -= (int64_t)86400000LL;
                }
                else if (measured_offset < -HALF_DAY) {
                    measured_offset += (int64_t)86400000LL;
                }

                gps_offset_update_ms(measured_offset);

                // Optional: log time sync occasionally (very spammy otherwise)
                // static uint32_t dbg = 0;
                // if ((dbg++ % 200) == 0) {
                //     char buf[96];
                //     int n = snprintf(buf, sizeof(buf), "GPS offset_ms=%lld", (long
                //     long)g_gps_time_offset_ms); if (n > 0)
                //     log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA, buf, 1,
                //     (size_t)n + 1);
                // }
            } else {
                // GPS has no fix - log occasionally
                if (++no_fix_counter >= 100) {
                    const char no_fix_txt[] = "GPS FATAL: No fix";
                    log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA, 
                                                no_fix_txt, 
                                                1,
                                                sizeof(no_fix_txt));

                    no_fix_counter = 0;
                }
            }
        } else {
            consecutive_errors++;

            if (consecutive_errors >= 5) {
                const char *error_type;

                switch (status) {
                    case NEOM9N_TIMEOUT:
                        error_type = "GPS ERROR: Timeout";
                        break;
                    case NEOM9N_SPI_ERR:
                        error_type = "GPS ERROR: SPI Error";
                        break;
                    case NEOM9N_PARSE_ERR:
                        error_type = "GPS ERROR: Parse Error";
                        break;
                    default:
                        error_type = "GPS ERROR: Undefined";
                        break;
                }

                log_telemetry_asynchronous(SEDS_DT_GENERIC_ERROR, 
                                            error_type, 
                                            1,
                                            strlen(error_type) + 1);

                consecutive_errors = 0;
            }
        }
        tx_thread_sleep(1);
    }
}

void create_neom9n_thread(void) {
    UINT status = tx_thread_create(
        &neom9n_thread, 
        "NEOM9N Thread", 
        neom9n_thread_entry, 
        0,
        neom9n_thread_stack, 
        NEOM9N_THREAD_STACK_SIZE, 
        5, 
        5,
        TX_NO_TIME_SLICE,   
        TX_AUTO_START
    );

    if (status != TX_SUCCESS) {
        die("Failed to create neom9n thread: %u", (unsigned)status);
    }
}
