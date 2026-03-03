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


#ifndef GPS_TEST_MODE
// Set to 1 to force GPS test mode (no hardware required).
#define GPS_TEST_MODE 0
#endif

#ifndef GPS_TEST_AUTO_FALLBACK
// If 1, automatically enter test mode after repeated SPI/timeout errors.
#define GPS_TEST_AUTO_FALLBACK 0
#endif

#ifndef GPS_TEST_FALLBACK_AFTER_ERRORS
#define GPS_TEST_FALLBACK_AFTER_ERRORS 50u
#endif

#ifndef GPS_TEST_FALLBACK_EPOCH_MS
// Feb 23, 2026 00:00:00 UTC (same value you used earlier)
#define GPS_TEST_FALLBACK_EPOCH_MS 1771804800000ULL
#endif

#ifndef GPS_TEST_LAT
#define GPS_TEST_LAT 42.6526f
#endif
#ifndef GPS_TEST_LON
#define GPS_TEST_LON -73.7562f
#endif
#ifndef GPS_TEST_ALT_M
#define GPS_TEST_ALT_M 100.0f
#endif

// Set to 1 to enable config, 0 to disable
#define GPS_ENABLE_RUNTIME_CONFIG 0

#define UNUSED_FUNCTION __attribute__((unused))

// External SPI handle
extern SPI_HandleTypeDef hspi1;

#ifndef TX_TIMER_TICKS_PER_SECOND
#error "TX_TIMER_TICKS_PER_SECOND must be defined by ThreadX."
#endif

/* ThreadX tick -> ms, using integer math */
static uint64_t tx_now_ms(void) {
    ULONG ticks = tx_time_get();
    return ((uint64_t)(uint32_t)ticks * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

static uint64_t wrap_day_ms(uint64_t ms) {
    const uint64_t DAY_MS = 86400000ULL;
    return ms % DAY_MS;
}

static UNUSED_FUNCTION const char *neom9n_status_to_string(NEOM9N_status_t status) {
    switch (status) {
        case NEOM9N_OK:
            return "OK";
        case NEOM9N_TIMEOUT:
            return "TIMEOUT";
        case NEOM9N_SPI_ERR:
            return "SPI_ERR";
        case NEOM9N_PARSE_ERR:
            return "PARSE_ERR";
        default:
            return "UNKNOWN";
    }
}

/* Stack + TCB for neom9n thread */
TX_THREAD neom9n_thread;
#define NEOM9N_THREAD_STACK_SIZE (9 * 1024u)
void neom9n_thread_entry(ULONG initial_input)
{
    (void)initial_input;
    HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, GPIO_PIN_SET);

    tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND * 3);  // Wait for GPS boot
    
#if GPS_ENABLE_RUNTIME_CONFIG
    // RAM only config
    if (gps_config(&hspi1, 3000)) {
        printf("GPS configured: GGA+RMC enabled, others disabled\n");
    } else {
        printf("Config failed\n");  // GPS still works, just more verbose
    }
#endif

    // Initialize GPS packet
    NEOM9N_t gps_packet;
    gps_init(&gps_packet, &hspi1);

    // Binary buffers for telemetry
    uint8_t gps_data_buffer[12]; // 3 floats (lat, lon, alt)

    // GPS time in milliseconds since UNIX epoch
    uint64_t gps_epoch_ms = 0;

    // Error tracking
    uint32_t consecutive_errors = 0;
    uint32_t no_fix_counter = 0;

    uint8_t gps_test_active = (GPS_TEST_MODE ? 1u : 0u);
    uint32_t gps_error_accum = 0;
    // HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, GPIO_PIN_SET);

    for (;;)
    {
        if (gps_test_active)
        {
            // Synthetic epoch time that advances with local ticks
            const uint64_t now_local_ms = tx_now_ms();
            gps_epoch_ms = GPS_TEST_FALLBACK_EPOCH_MS + now_local_ms;

            // Publish a sane GPS offset for other subsystems
            const uint64_t local_tod = wrap_day_ms(now_local_ms);
            const uint64_t gps_tod = wrap_day_ms(gps_epoch_ms);
            const int64_t measured_offset = (int64_t)gps_tod - (int64_t)local_tod;
            gps_offset_update_ms(measured_offset);

            // Provide unix time base (only used on TELEMETRY_TIME_MASTER builds)
            telemetry_set_unix_time_ms(gps_epoch_ms);

            // Emit a fixed GPS position payload occasionally
            float fake[3] = {GPS_TEST_LAT, GPS_TEST_LON, GPS_TEST_ALT_M};
            log_telemetry_asynchronous(SEDS_DT_GPS_DATA, fake, 3, sizeof(float));
            printf("GPS TEST MODE: emitted fake GPS data\r\n");
            tx_thread_sleep(200);

            continue;
        }

        NEOM9N_status_t status = receive_nmea(&gps_packet, NMEA_MAX_WAIT, NMEA_MAX_IGNORES);

        if (status == NEOM9N_OK)
        {
            consecutive_errors = 0;
            gps_error_accum = 0;

            if (gps_has_fix(&gps_packet))
            {
                no_fix_counter = 0;

                // Pack and send GPS position data (lat, lon, alt)
                pack_gps_data(&gps_packet, gps_data_buffer);
                //log_telemetry_asynchronous(SEDS_DT_GPS_DATA,
                //                           gps_data_buffer,
                //                           3,
                //                           sizeof(float));
                /* Print GPS data using fixed-point formatting because floating
                 * point printf (%f) is disabled. Lat/lon use 6 fractional
                 * digits, altitude uses 2 fractional digits.
                 */
                {
                    int32_t lat_int = (int32_t)(gps_packet.lat * 1000000.0f);
                    int32_t lon_int = (int32_t)(gps_packet.lon * 1000000.0f);
                    int32_t alt_int = (int32_t)(gps_packet.altitude_msl * 100.0f);
                    
                    printf("========== GPS DATA ==========\n");
                    
                    // Position
                    printf("Position:\n");
                    printf("  Lat: %d.%06d° %c\n", 
                           (int)(lat_int / 1000000), 
                           (int)(lat_int >= 0 ? lat_int % 1000000 : -(lat_int % 1000000)),
                           gps_packet.lat >= 0 ? 'N' : 'S');
                    printf("  Lon: %d.%06d° %c\n", 
                           (int)(lon_int / 1000000), 
                           (int)(lon_int >= 0 ? lon_int % 1000000 : -(lon_int % 1000000)),
                           gps_packet.lon >= 0 ? 'E' : 'W');
                    printf("  Alt: %d.%02d m MSL\n", 
                           (int)(alt_int / 100), 
                           (int)(alt_int >= 0 ? alt_int % 100 : -(alt_int % 100)));
                    
                    // Date
                    printf("Date: %02d/%02d/20%02d\n", 
                           gps_packet.day, gps_packet.month, gps_packet.year);
                    
                    // Time
                    printf("Time: %02d:%02d:%02d.%03d UTC\n", 
                           gps_packet.hours, gps_packet.minutes, gps_packet.seconds, gps_packet.milliseconds);

                    // Unix Epoch
                    gps_epoch_ms = get_datetime_data(&gps_packet);
                    printf("FC Time: %llu ms since 1/1/1970\n", gps_epoch_ms);
                    
                    // Status
                    printf("Status:\n");
                    printf("  Fix: %s\n", gps_packet.valid_fix ? "VALID" : "NO FIX");
                    printf("  Last Update: %lu ticks\n", (unsigned long)gps_packet.last_update_tick);
                    
                    printf("==============================\n");
                }
                // Pack GPS time-of-day.
                // IMPORTANT: pack_time_data() must fill gps_time_of_day_ms as *ms since
                // midnight UTC* (or you should change it accordingly).

                // Pack GPS time-of-day (ms since UNIX Epoch) from the parsed fields.
                gps_epoch_ms = get_datetime_data(&gps_packet);

                // Update global offset so other threads can compute GPS time-of-day
                // from local ticks. offset := gps_tod_ms - local_tod_ms
                const uint64_t local_ms = tx_now_ms();
                const uint64_t local_tod = wrap_day_ms(local_ms);
                const uint64_t gps_tod  = wrap_day_ms(gps_epoch_ms);

                // Compute signed difference (gps - local)
                int64_t measured_offset = (int64_t)gps_tod - (int64_t)local_tod;
                gps_offset_update_ms(measured_offset);

                // Inform telemetry master of the current unix epoch time so
                // time announcements use a valid unix base.
                telemetry_set_unix_time_ms(gps_epoch_ms);

            }
            else
            {
                // GPS has no fix - log occasionally
                if (++no_fix_counter >= 100)
                {
                    printf("GPS has no fix\r\n");
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
            consecutive_errors ++;
            gps_error_accum++;

#if GPS_TEST_AUTO_FALLBACK
            if (gps_error_accum >= (uint32_t)GPS_TEST_FALLBACK_AFTER_ERRORS)
            {
                gps_test_active = 1u;
                const char fb_txt[] = "GPS TEST MODE: entering fallback (no GPS detected)";
                log_telemetry_asynchronous(SEDS_DT_WARNING, fb_txt, sizeof(fb_txt), 1);
                continue;
            }
#endif

            if (consecutive_errors >= 5) {
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

                // log_telemetry_asynchronous(
                //     SEDS_DT_WARNING,
                //     error_type,
                //     strlen(error_type) + 1, // include NUL for C-string payload
                //     1);
                printf("%s\r\n", error_type); 
                printf("%u",gps_error_accum);

                consecutive_errors = 0;
                gps_error_accum = 0;
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
