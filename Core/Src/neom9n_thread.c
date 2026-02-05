// neom9n_thread.c
#include "RF-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "neom9n.h"

// External SPI handle (defined in stm32g4xx_hal_msp.c)
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
                                sizeof(started_txt), 
                                1); //Inital log statement

    NEOM9N_t packet = {
        .hspi = &hspi1,
        .lat = 0.0f,
        .lon = 0.0f
    };
    const char created_txt[] = "NEOM9N packet successfully allocated";
    log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA, 
                                created_txt,
                                sizeof(created_txt),
                                1); //Log succesasful packet allocation

    for (;;) {
        NEOM9N_status_t response = receive_nmea(&packet, NMEA_MAX_WAIT, NMEA_MAX_IGNORES);
        if (response != NEOM9N_OK) {
            const char error_txt[] = "ERROR parsing NEOM9N GPS data";
            log_telemetry_asynchronous(SEDS_DT_GENERIC_ERROR,
                                        error_txt,
                                        sizeof(error_txt),
                                        1); //Error log statement
        }

        char position_txt[40]; 
        float_to_str(packet.lat, position_txt, NEOM9N_PRECISION);
        int len = strlen(position_txt);
        position_txt[len++] = ',';
        float_to_str(packet.lon, position_txt + len, NEOM9N_PRECISION);

        log_telemetry_asynchronous(SEDS_DT_GPS_DATA,
                                    position_txt,
                                    sizeof(position_txt),
                                    1); //Log position data

        tx_thread_sleep(10);  // 10 ticks sleep - adjust as needed
    }
}

void create_neom9n_thread(void) 
{
    UINT status = tx_thread_create(
        &neom9n_thread,
        "NEOM9N Thread",
        neom9n_thread_entry,
        0,    // initial input
        neom9n_thread_stack,
        NEOM9N_THREAD_STACK_SIZE,
        5,    // priority
        5,    // preemption threshold
        TX_NO_TIME_SLICE,
        TX_AUTO_START);

    if (status != TX_SUCCESS) {
        die("Failed to create neom9n thread: %u", (unsigned)status);
    }
}
