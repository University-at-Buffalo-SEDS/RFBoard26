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
 * MODES:
 *      TELEMETRY_TEST_MODE - Outputs dumby gps data for system testing 
 *      GPS_TEST_MODE - Outputs GPS data to serial and interfaces with a custom map software
 *      REMOVED: GPS_TEST_AUTO_FALLBACK - Enters test mode after repeated errors
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


/* -- TELEMETRY TEST MODE --------------------------- */
#ifndef TELEMETRY_TEST_MODE
// Set to 1 to force TELEMETRY test mode
#define TELEMETRY_TEST_MODE 0
#endif

#if TELEMETRY_TEST_MODE
#ifndef TELEMETRY_TEST_EPOCH_MS
// Feb 23, 2026 00:00:00 UTC
#define TELEMETRY_TEST_EPOCH_MS 1771804800000ULL
#endif

#ifndef TELEMETRY_TEST_LAT
#define TELEMETRY_TEST_LAT 42.6526f
#endif
#ifndef TELEMETRY_TEST_LON
#define TELEMETRY_TEST_LON -73.7562f
#endif
#ifndef TELEMETRY_TEST_ALT_M
#define TELEMETRY_TEST_ALT_M 100.0f
#endif
#endif


/* -- GPS TEST MODE --------------------------------- */
#ifndef GPS_TEST_MODE
// Set to 1 to force GPS test mode 
#define GPS_TEST_MODE 0
#endif

/* -- PRODUCTION MODE ------------------------------- */
#define PRODUCTION_MODE ((!TELEMETRY_TEST_MODE) && (!GPS_TEST_MODE))


/* -- External SPI handle --------------------------- */
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

/* Stack + TCB for neom9n thread */
TX_THREAD neom9n_thread;
#define NEOM9N_THREAD_STACK_SIZE (9 * 1024u)

void neom9n_thread_entry(ULONG initial_input)
{
    (void)initial_input;
    HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, GPIO_PIN_SET);

    tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND * 5);  // Wait for GPS boot

#if GPS_TEST_MODE
    tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND * 5);  // Give user enoguh time to connect to serial out and log output window
#endif

    /* RAM only config - Needs to be set each time but is safer */
    if (gps_config(&hspi1, 3000)) {
        const char flashed_txt[] = "GPS configured: GGA+RMC enabled -> others disabled";
        #if GPS_TEST_MODE
            printf("%s\r\n", flashed_txt);
        #endif
        #if PRODUCTION_MODE
            log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA,
                                        flashed_txt,
                                        strlen(flashed_txt) + 1,    // include NUL for C-string payload
                                        1);
        #endif
    } else {
        const char failed_txt[] = "Config failed";
        #if GPS_TEST_MODE
            printf("%s\r\n", failed_txt); 
        #endif 
        #if PRODUCTION_MODE
            log_telemetry_asynchronous(SEDS_DT_WARNING,
                                        failed_txt,
                                        strlen(failed_txt) + 1,    // include NUL for C-string payload
                                        1);
        #endif
    }   // GPS still works, just more verbose

    // Initialize GPS packet
    NEOM9N_t gps_packet;
    gps_init(&gps_packet, &hspi1);

    // Binary buffers for telemetry
    float gps_data_buffer[3]; // 3 floats (lat, lon, alt)

    // GPS time in milliseconds since UNIX epoch
    uint64_t gps_epoch_ms = 0;

    // Error tracking
    uint32_t consecutive_errors = 0;
    uint32_t no_fix_counter = 0;
    // HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, GPIO_PIN_SET);

    for (;;)
    {
#if TELEMETRY_TEST_MODE
            // Synthetic epoch time that advances with local ticks
            const uint64_t now_local_ms = tx_now_ms();
            gps_epoch_ms = TELEMETRY_TEST_EPOCH_MS + now_local_ms;

            // Publish a sane GPS offset for other subsystems
            const uint64_t local_tod = wrap_day_ms(now_local_ms);
            const uint64_t gps_tod = wrap_day_ms(gps_epoch_ms);
            const int64_t measured_offset = (int64_t)gps_tod - (int64_t)local_tod;
            gps_offset_update_ms(measured_offset);

            // Feed the telemetry library's network clock directly from GPS epoch time.
            telemetry_set_unix_time_ms(gps_epoch_ms);

            // Emit a fixed GPS position payload occasionally
            float fake[3] = {TELEMETRY_TEST_LAT, TELEMETRY_TEST_LON, TELEMETRY_TEST_ALT_M};
            log_telemetry_asynchronous(SEDS_DT_GPS_DATA, fake, 3, sizeof(float));
            printf("GPS TEST MODE: emitted fake GPS data\r\n");
            tx_thread_sleep(200);

            continue;
#endif
        NEOM9N_status_t status = receive_nmea(&gps_packet, NMEA_MAX_WAIT, NMEA_MAX_IGNORES);

        if (status == NEOM9N_OK)
        {
            consecutive_errors = 0;

            if (gps_has_fix(&gps_packet))
            {
                no_fix_counter = 0;

                /* Pack and send GPS position data (lat, lon, alt) */
                pack_gps_data(&gps_packet, gps_data_buffer);
#if PRODUCTION_MODE
                log_telemetry_asynchronous(SEDS_DT_GPS_DATA,
                                           gps_data_buffer,
                                           3,
                                           sizeof(float));
#endif
                /** 
                 * Pack GPS time-of-day.
                 * IMPORTANT: pack_time_data() must fill gps_time_of_day_ms as *ms since
                 * midnight UTC* (or you should change it accordingly).
                 */ 

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

                // Feed the telemetry library's network clock directly from
                // the parsed GPS epoch time.
#if PRODUCTION_MODE
                telemetry_set_unix_time_ms(gps_epoch_ms); //send time 
                log_telemetry_asynchronous(SEDS_DT_GPS_SATELLITE_NUMBER, 
                                            &gps_packet.num_satellites, 
                                            1, 
                                            sizeof(uint8_t));  // send N sats
#endif
#if GPS_TEST_MODE
                /** 
                 * Print GPS data using fixed-point formatting because floating
                 * point printf (%f) is disabled. Lat/lon use 6 fractional
                 * digits, altitude uses 2 fractional digits.
                 */
                int32_t lat_int = (int32_t)(gps_packet.lat * 1000000.0f);
                int32_t lon_int = (int32_t)(gps_packet.lon * 1000000.0f);
                int32_t alt_int = (int32_t)(gps_packet.altitude_msl * 100.0f);
                
                printf("========== GPS DATA ==========\n");
                
                /* Position */
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
                
                /* Date */
                printf("Date: %02d/%02d/20%02d\n", 
                        gps_packet.day, gps_packet.month, gps_packet.year);
                
                /* Time */
                printf("Time: %02d:%02d:%02d.%03d UTC\n", 
                        gps_packet.hours, gps_packet.minutes, gps_packet.seconds, gps_packet.milliseconds);

                /* Unix Epoch */
                uint32_t epoch_hi = (uint32_t)(gps_epoch_ms / 1000000000ULL);
                uint32_t epoch_lo = (uint32_t)(gps_epoch_ms % 1000000000ULL);
                printf("FC Time: %lu%09lu ms since 1/1/1970\n", (unsigned long)epoch_hi, (unsigned long)epoch_lo);    

                /* Status */
                printf("Status:\n");
                printf("  Satellites: %d\n", gps_packet.num_satellites);
                printf("  Fix: %s\n", gps_packet.valid_fix ? "VALID" : "NO FIX");
                printf("  Last Update: %lu ticks\n", (unsigned long)gps_packet.last_update_tick);
                
                printf("==============================\n");
#endif
            }
            else
            {
                // GPS has no fix - log occasionally
                if (++no_fix_counter >= 100)
                {
#if GPS_TEST_MODE
                    printf("GPS has no fix\r\n");
#endif
#if PRODUCTION_MODE
                    const char no_fix_txt[] = "GPS FATAL: No fix";
                    log_telemetry_asynchronous(SEDS_DT_WARNING,
                                               no_fix_txt,
                                               strlen(no_fix_txt), 
                                               1);
#endif
                    no_fix_counter = 0;
                }
            }
        }
        else
        {
            consecutive_errors ++;

            if (consecutive_errors >= 20) {
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
#if PRODUCTION_MODE
                log_telemetry_asynchronous(SEDS_DT_WARNING,
                                            error_type,
                                            strlen(error_type) + 1, // include NUL for C-string payload
                                            1);
#endif
#if GPS_TEST_MODE
                printf("%s\r\n", error_type);
#endif
                consecutive_errors = 0;
            }
        }
        tx_thread_sleep(50);
    }
}

UINT create_neom9n_thread(TX_BYTE_POOL *byte_pool)
{
    CHAR *pointer;

    /* Allocate the stack for test */
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
