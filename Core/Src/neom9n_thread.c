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

#define michael (;;)

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
volatile uint8_t g_neom9n_has_fix = 0U;
#define NEOM9N_THREAD_STACK_SIZE (9 * 1024u)
#define GPS_LINK_WARNING_INTERVAL_MS (5ULL * 60ULL * 1000ULL)
#define GPS_DATA_LOG_INTERVAL_MS 750ULL
#define GPS_SATELLITE_LOG_INTERVAL_MS 1000ULL
#define GPS_NO_DATA_SATELLITE_LOG_INTERVAL_MS 5000ULL
#define GPS_SATELLITE_STALE_MS 5000UL
#define GPS_ERROR_LOG_INTERVAL_MS (5ULL * 60ULL * 1000ULL)
#define GPS_CONSECUTIVE_ERROR_THRESHOLD 20U
#define GPS_SPI_RECOVERY_ERROR_THRESHOLD 3U
#define GPS_SPI_RECOVERY_INTERVAL_MS 5000ULL
#define GPS_NO_FIX_ERROR_THRESHOLD 100U

static void gps_emit_satellite_count(uint8_t satellite_count,
                                     uint64_t now_ms,
                                     uint64_t interval_ms,
                                     uint64_t *next_emit_ms) {
#if PRODUCTION_MODE
    static bool have_last_satellite_count = false;
    static uint8_t last_satellite_count = 0U;
    const bool changed = !have_last_satellite_count ||
                         last_satellite_count != satellite_count;

    if (!changed && now_ms < *next_emit_ms) {
        return;
    }

    SedsResult result = log_telemetry_asynchronous(SEDS_DT_GPS_SATELLITE_NUMBER,
                                                   &satellite_count,
                                                   1,
                                                   sizeof(uint8_t));
    (void)result;
    have_last_satellite_count = true;
    last_satellite_count = satellite_count;
    *next_emit_ms = now_ms + interval_ms;
#else
    (void)satellite_count;
    (void)now_ms;
    (void)interval_ms;
    (void)next_emit_ms;
#endif
}

static uint8_t gps_recent_satellite_count_or_zero(const NEOM9N_t *packet) {
    if (packet == NULL || packet->last_satellite_update_tick == 0U) {
        return 0U;
    }

    if ((uint32_t)(HAL_GetTick() - packet->last_satellite_update_tick) >
        (uint32_t)GPS_SATELLITE_STALE_MS) {
        return 0U;
    }

    return packet->num_satellites;
}

static void gps_log_initial_link_warning(void) {
#if GPS_TEST_MODE
    printf("GPS WARNING: We have not yet established a link\r\n");
#endif
#if PRODUCTION_MODE
    const char no_link_txt[] = "GPS WARNING: We have not yet established a link";
    log_telemetry_asynchronous(SEDS_DT_WARNING,
                                no_link_txt,
                                strlen(no_link_txt) + 1,
                                1);
#endif
}

static void gps_log_no_fix_warning(void) {
#if GPS_TEST_MODE
    printf("GPS has no fix\r\n");
#endif
#if PRODUCTION_MODE
    const char no_fix_txt[] = "GPS FATAL: No fix";
    log_telemetry_asynchronous(SEDS_DT_WARNING,
                               no_fix_txt,
                               strlen(no_fix_txt) + 1,
                               1);
#endif
}

static size_t gps_error_index(NEOM9N_status_t status) {
    switch (status)
    {
    case NEOM9N_TIMEOUT:
        return 0U;
    case NEOM9N_SPI_ERR:
        return 1U;
    case NEOM9N_PARSE_ERR:
        return 2U;
    default:
        return 3U;
    }
}

static const char *gps_error_text(NEOM9N_status_t status) {
    switch (status)
    {
    case NEOM9N_TIMEOUT:
        return "GPS ERROR: Timeout";
    case NEOM9N_SPI_ERR:
        return "GPS ERROR: SPI Error";
    case NEOM9N_PARSE_ERR:
        return "GPS ERROR: Parse Error";
    default:
        return "GPS ERROR: Undefined";
    }
}

static bool gps_recover_spi(NEOM9N_t *packet) {
    if (packet == NULL || packet->hspi == NULL) {
        return false;
    }

    NEOGPS_CS_HIGH();
    (void)HAL_SPI_Abort(packet->hspi);
    (void)HAL_SPI_DeInit(packet->hspi);
    tx_thread_sleep(1U);

    if (HAL_SPI_Init(packet->hspi) != HAL_OK) {
        NEOGPS_CS_HIGH();
        return false;
    }

    NEOGPS_CS_HIGH();
    return gps_config(packet->hspi, 1000U);
}

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
#if GPS_TEST_MODE
        const char failed_txt[] = "Config failed";
        printf("%s\r\n", failed_txt);
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
    bool gps_link_established = false;
    bool gps_initial_link_warning_sent = false;
    bool gps_no_fix_warning_sent = false;
    uint64_t next_initial_link_warning_ms = tx_now_ms() + GPS_LINK_WARNING_INTERVAL_MS;
    uint64_t next_gps_data_log_ms = 0ULL;
    uint64_t next_gps_satellite_log_ms = 0ULL;
    uint64_t next_no_gps_data_satellite_log_ms = 0ULL;
    uint64_t next_gps_error_log_ms[4] = {0ULL, 0ULL, 0ULL, 0ULL};
    uint64_t next_gps_spi_recover_ms = 0ULL;
    // HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin, GPIO_PIN_SET);

    for michael
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
            if (now_local_ms >= next_gps_data_log_ms) {
                float fake[3] = {TELEMETRY_TEST_LAT, TELEMETRY_TEST_LON, TELEMETRY_TEST_ALT_M};
                log_telemetry_asynchronous(SEDS_DT_GPS_DATA, fake, 3, sizeof(float));
                printf("GPS TEST MODE: emitted fake GPS data\r\n");
                next_gps_data_log_ms = now_local_ms + GPS_DATA_LOG_INTERVAL_MS;
            }
            tx_thread_sleep(50);

            continue;
#endif
        NEOM9N_status_t status = receive_nmea(&gps_packet, NMEA_MAX_WAIT, NMEA_MAX_IGNORES);

        if (status == NEOM9N_OK)
        {
            consecutive_errors = 0;

            if (gps_has_fix(&gps_packet))
            {
                const uint64_t local_ms = tx_now_ms();

                g_neom9n_has_fix = 1U;
                gps_link_established = true;
                gps_no_fix_warning_sent = false;
                no_fix_counter = 0;

                if (gps_datetime_valid(&gps_packet)) {
                    // Convert the parsed GPS date/time to Unix epoch ms before emitting
                    // telemetry so production packets are stamped with GPS-backed time.
                    gps_epoch_ms = get_datetime_data(&gps_packet);

                    // Update global offset so other threads can compute GPS time-of-day
                    // from local ticks. offset := gps_tod_ms - local_tod_ms
                    const uint64_t local_tod = wrap_day_ms(local_ms);
                    const uint64_t gps_tod  = wrap_day_ms(gps_epoch_ms);

                    // Compute signed difference (gps - local)
                    int64_t measured_offset = (int64_t)gps_tod - (int64_t)local_tod;
                    gps_offset_update_ms(measured_offset);

                    // Feed the telemetry library's network clock directly from
                    // the parsed GPS epoch time before any production telemetry logs.
#if PRODUCTION_MODE
                    telemetry_set_unix_time_ms(gps_epoch_ms);
#endif
                }

                /* Pack and send GPS position data (lat, lon, alt) */
                pack_gps_data(&gps_packet, gps_data_buffer);
                const bool gps_data_log_due = (local_ms >= next_gps_data_log_ms);
#if PRODUCTION_MODE
                if (gps_data_log_due) {
                    log_telemetry_asynchronous(SEDS_DT_GPS_DATA,
                                               gps_data_buffer,
                                               3,
                                               sizeof(float));
                }
#endif
                /** 
                 * Pack GPS time-of-day.
                 * IMPORTANT: pack_time_data() must fill gps_time_of_day_ms as *ms since
                 * midnight UTC* (or you should change it accordingly).
                 */ 

#if PRODUCTION_MODE
                gps_emit_satellite_count(gps_packet.num_satellites,
                                         local_ms,
                                         GPS_SATELLITE_LOG_INTERVAL_MS,
                                         &next_gps_satellite_log_ms);  // send N sats
#endif
#if GPS_TEST_MODE
                if (gps_data_log_due) {
                /** 
                 * Print GPS data using fixed-point formatting because floating
                 * point printf (%f) is disabled. Lat/lon use 6 fractional
                 * digits, altitude uses 2 fractional digits.
                 */
                int32_t lat_int = (int32_t)(gps_packet.lat * 1000000.0f);
                int32_t lon_int = (int32_t)(gps_packet.lon * 1000000.0f);
                int32_t alt_int = (int32_t)(gps_packet.altitude_msl * 100.0f);
              
                printf("\x1B[2J\x1B[H");

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
                }
#endif
                if (gps_data_log_due) {
                    next_gps_data_log_ms = local_ms + GPS_DATA_LOG_INTERVAL_MS;
                }
            }
            else
            {
                const uint64_t now_ms = tx_now_ms();
                g_neom9n_has_fix = 0U;
                gps_emit_satellite_count(gps_recent_satellite_count_or_zero(&gps_packet),
                                         now_ms,
                                         GPS_NO_DATA_SATELLITE_LOG_INTERVAL_MS,
                                         &next_no_gps_data_satellite_log_ms);

                if (!gps_link_established)
                {
                    if (!gps_initial_link_warning_sent && now_ms >= next_initial_link_warning_ms)
                    {
                        gps_log_initial_link_warning();
                        gps_initial_link_warning_sent = true;
                    }
                }
                // GPS lost fix after its initial link - log once per lost-lock episode.
                else if (!gps_no_fix_warning_sent &&
                         ++no_fix_counter >= GPS_NO_FIX_ERROR_THRESHOLD)
                {
                    gps_log_no_fix_warning();
                    gps_no_fix_warning_sent = true;
                    no_fix_counter = 0;
                }
            }
        }
        else
        {
            const uint64_t now_ms = tx_now_ms();
            if (consecutive_errors < GPS_CONSECUTIVE_ERROR_THRESHOLD) {
                consecutive_errors++;
            }
            g_neom9n_has_fix = 0U;
            gps_emit_satellite_count(gps_recent_satellite_count_or_zero(&gps_packet),
                                     now_ms,
                                     GPS_NO_DATA_SATELLITE_LOG_INTERVAL_MS,
                                     &next_no_gps_data_satellite_log_ms);

            if (status == NEOM9N_SPI_ERR &&
                consecutive_errors >= GPS_SPI_RECOVERY_ERROR_THRESHOLD &&
                now_ms >= next_gps_spi_recover_ms) {
                (void)gps_recover_spi(&gps_packet);
                next_gps_spi_recover_ms = now_ms + GPS_SPI_RECOVERY_INTERVAL_MS;
            }

            if (!gps_link_established)
            {
                if (!gps_initial_link_warning_sent && now_ms >= next_initial_link_warning_ms)
                {
                    gps_log_initial_link_warning();
                    gps_initial_link_warning_sent = true;
                }
            }
            else if (consecutive_errors >= GPS_CONSECUTIVE_ERROR_THRESHOLD) {
                const char *error_type = gps_error_text(status);
                const size_t error_index = gps_error_index(status);

                if (now_ms >= next_gps_error_log_ms[error_index]) {
#if PRODUCTION_MODE
                    log_telemetry_asynchronous(SEDS_DT_WARNING,
                                                error_type,
                                                strlen(error_type) + 1, // include NUL for C-string payload
                                                1);
#endif
#if GPS_TEST_MODE
                    printf("%s\r\n", error_type);
#endif
                    next_gps_error_log_ms[error_index] = now_ms + GPS_ERROR_LOG_INTERVAL_MS;
                }
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
