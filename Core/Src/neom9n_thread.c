// neom9n_thread.c
#include "RF-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "neom9n.h"

// External SPI handle
extern SPI_HandleTypeDef hspi1;

// Stack + TCB for neom9n thread
TX_THREAD neom9n_thread;
#define NEOM9N_THREAD_STACK_SIZE 1024u
ULONG neom9n_thread_stack[NEOM9N_THREAD_STACK_SIZE / sizeof(ULONG)];

void neom9n_thread_entry(ULONG initial_input) 
{
    (void)initial_input;
    const char started_txt[] = "NEOM9N thread starting";
    log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA, 
                                started_txt, 
                                1, 
                                sizeof(started_txt)); //Inital log statement

    
    // Initialize GPS packet
    NEOM9N_t gps_packet;
    gps_init(&gps_packet, &hspi1);
    
    const char init_txt[] = "NEOM9N packet initialized";
    log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA, 
                                init_txt,
                                1,
                                sizeof(init_txt));
    
    // Binary buffers for telemetry
    uint8_t gps_data_buffer[12];   // 3 floats (lat, lon, alt)
    uint8_t gps_time_buffer[8];    // 1 double (seconds since midnight)

    // Error tracking
    uint32_t consecutive_errors = 0;
    uint32_t no_fix_counter = 0;
    
    for (;;) {
        // Receive and parse NMEA sentence
        NEOM9N_status_t status = receive_nmea(&gps_packet, 
                                                NMEA_MAX_WAIT, 
                                                NMEA_MAX_IGNORES);
        
        if (status == NEOM9N_OK) {
            consecutive_errors = 0;
            
            // Only log if we have a valid fix
            if (gps_has_fix(&gps_packet)) {
                no_fix_counter = 0;
                
                // Pack and send GPS position data (lat, lon, alt)
                pack_gps_data(&gps_packet, gps_data_buffer);
                log_telemetry_asynchronous(SEDS_DT_GPS_DATA,
                                            gps_data_buffer,
                                            3, 
                                            sizeof(float));
                
                // Pack and send GPS time data (seconds since midnight UTC)
                pack_time_data(&gps_packet, gps_time_buffer);
                log_telemetry_asynchronous(SEDS_DT_GPS_TIME,
                                            gps_time_buffer,
                                            1,   
                                            sizeof(double));
            } 
            else {
                // GPS has no fix - log occasionally
                if (++no_fix_counter >= 100) {  // Every 100 iterations
                    const char no_fix_txt[] = "GPS FATAL: No fix";
                    log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA,
                                                no_fix_txt,
                                                1,
                                                sizeof(no_fix_txt));
                    no_fix_counter = 0;
                }
            }
        } 
        else {
            consecutive_errors++;  // Error occurred
            
            // Only log after multiple consecutive errors
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
                
                consecutive_errors = 0;  // Reset after logging
            }
        }
        tx_thread_sleep(1);  // 1 tick; adjust as needed
    }
}

void create_neom9n_thread(void) 
{
    UINT status = tx_thread_create(
        &neom9n_thread,
        "NEOM9N Thread",
        neom9n_thread_entry,
        0,    // initial input *
        neom9n_thread_stack,
        NEOM9N_THREAD_STACK_SIZE,
        5,    // priority *
        5,    // preemption threshold *
        TX_NO_TIME_SLICE,
        TX_AUTO_START); // * = idk what I should put here change later

    if (status != TX_SUCCESS) {
        die("Failed to create neom9n thread: %u", (unsigned)status);
    }
}