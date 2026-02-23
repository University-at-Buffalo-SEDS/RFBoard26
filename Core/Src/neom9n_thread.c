/**
 * neom9n_thread.c
 *
 * UPDATED:
 *  - Maintains a globally-accessible GPS-synced offset (ms) that other threads
 *    (telemetry thread)
 *    can read to convert local ThreadX-relative time into GPS time-of-day.
 *  - OFFSET:
 *       gps_time_of_day_ms ≈ (local_tx_now_ms % 86400000) + g_gps_time_offset_ms
 *
 * NOTE:
 *  - This is "time-of-day" sync (seconds since midnight UTC), because your
 *    current GPS time packet
 *    appears to be "seconds since midnight". If you later expose full date
 *    (Y/M/D) or Unix epoch, we can make this a true epoch offset.
 *
 * EXAMPLE: How telemetry thread should use it
 *    extern volatile int64_t g_gps_time_offset_ms;
 *    extern volatile uint32_t g_gps_time_offset_valid;
 *    if (g_gps_time_offset_valid) {
 *        uint64_t local_ms = tx_now_ms();
 *        uint64_t gps_tod_ms = (local_ms % 86400000ULL) +
 *        (int64_t)g_gps_time_offset_ms;
 *    }
 */

#include "RF-Threads.h"
#include "neom9n.h"
#include "neom9n_config.h"
#include "gps_time.h"
#include "telemetry.h"
#include "tx_api.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "can_bus.h"

// External SPI handle
extern SPI_HandleTypeDef hspi1;

#ifndef TX_TIMER_TICKS_PER_SECOND
#error "TX_TIMER_TICKS_PER_SECOND must be defined by ThreadX."
#endif

static uint64_t tx_now_ms(void)
{
    // ThreadX tick -> ms, using integer math.
    // If your tick rate does not divide 1000 evenly, this will truncate.
    // (Still fine for offset sync; if you want better precision, accumulate
    // remainder.)
    ULONG ticks = tx_time_get();
    return ((uint64_t)(uint32_t)ticks * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

static uint64_t wrap_day_ms(uint64_t ms)
{
    const uint64_t DAY_MS = 86400000ULL;
    return ms % DAY_MS;
}

// Stack + TCB for neom9n thread
TX_THREAD neom9n_thread;
#define NEOM9N_THREAD_STACK_SIZE (4 * 1024u)
ULONG neom9n_thread_stack[NEOM9N_THREAD_STACK_SIZE / sizeof(ULONG)];

void neom9n_thread_entry(ULONG initial_input)
{
    (void)initial_input;
    HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, GPIO_PIN_SET);
    ///**
    tx_thread_sleep(2000); // Wait 2 seconds for GPS to boot (recommended datasheet)

    // Configure for rocket flight
    // @note: Confirm with datasheet, after first config this block is likely reducdent
    if (config_gps_seds_rocket(&hspi1, 5000))
    {
        const char success[] = "NEOM9N config set successful";
        log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA,
                                   success,
                                   1,
                                   sizeof(success)); // Log config success
    }
    else
    {
        // Factory settings are fine, so no need to terminate if config fails
        // Config method greatly reduces overhead, but failure is not critical
        // Data rate unaffected by config success or failure
        const char failure[] = "NEOM9N config set failed";
        log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA,
                                   failure,
                                   1,
                                   sizeof(failure)); // Log config failure
    }

    tx_thread_sleep(100); // Wait for config to settle
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
    HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, GPIO_PIN_SET);

    for (;;)
    {

        NEOM9N_status_t status = receive_nmea(&gps_packet, NMEA_MAX_WAIT, NMEA_MAX_IGNORES);

        if (status == NEOM9N_OK)
        {
            consecutive_errors = 0;

            if (gps_has_fix(&gps_packet))
            {
                no_fix_counter = 0;

                // Pack and send GPS position data (lat, lon, alt)
                pack_gps_data(&gps_packet, gps_data_buffer);
                log_telemetry_asynchronous(SEDS_DT_GPS_DATA,
                                           gps_data_buffer,
                                           3,
                                           sizeof(float));
                /* Print GPS data using fixed-point formatting because floating
                 * point printf (%f) is disabled. Lat/lon use 6 fractional
                 * digits, altitude uses 2 fractional digits.
                 */
                {
                    double latf = gps_packet.lat;
                    double lonf = gps_packet.lon;
                    double altf = gps_packet.altitude_msl;

                    int64_t lat_scaled = (int64_t)(latf * 1000000.0 + (latf >= 0 ? 0.5 : -0.5));
                    int64_t lon_scaled = (int64_t)(lonf * 1000000.0 + (lonf >= 0 ? 0.5 : -0.5));
                    int64_t alt_scaled = (int64_t)(altf * 100.0 + (altf >= 0 ? 0.5 : -0.5));

                    int64_t lat_whole = lat_scaled / 1000000;
                    int64_t lat_frac = lat_scaled % 1000000;
                    if (lat_frac < 0)
                        lat_frac = -lat_frac;

                    int64_t lon_whole = lon_scaled / 1000000;
                    int64_t lon_frac = lon_scaled % 1000000;
                    if (lon_frac < 0)
                        lon_frac = -lon_frac;

                    int64_t alt_whole = alt_scaled / 100;
                    int64_t alt_frac = alt_scaled % 100;
                    if (alt_frac < 0)
                        alt_frac = -alt_frac;

                    printf("GPS data sent: lat=%lld.%06lld, lon=%lld.%06lld, alt=%lld.%02lld\n",
                           (long long)lat_whole, (long long)lat_frac,
                           (long long)lon_whole, (long long)lon_frac,
                           (long long)alt_whole, (long long)alt_frac);

                }
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
            }
            else
            {
                // GPS has no fix - log occasionally
                if (++no_fix_counter >= 100)
                {
                    printf("GPS has no fix\n");
                    const char no_fix_txt[] = "GPS FATAL: No fix";
                    log_telemetry_asynchronous(SEDS_DT_WARNING,
                                               no_fix_txt,
                                               sizeof(no_fix_txt), 1);

                    no_fix_counter = 0;
                }
            }
        }
        else
        {
            consecutive_errors++;

            if (consecutive_errors >= 5)
            {
                const char *error_type;

                switch (status)
                {
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

                log_telemetry_asynchronous(
                    SEDS_DT_WARNING,
                    error_type,
                    strlen(error_type) + 1, // include NUL for C-string payload
                    1);

                consecutive_errors = 0;
            }
        }
        tx_thread_sleep(50);
    }
}

UINT create_neom9n_thread(TX_BYTE_POOL *byte_pool)
{
    CHAR *pointer;

    /* Allocate the stack for test  */
    if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
                         NEOM9N_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }
    UINT status = tx_thread_create(
        &neom9n_thread,
        "NEOM9N Thread",
        neom9n_thread_entry,
        0,
        pointer,
        NEOM9N_THREAD_STACK_SIZE,
        4,
        4,
        TX_NO_TIME_SLICE,
        TX_AUTO_START);

    return status;
}
