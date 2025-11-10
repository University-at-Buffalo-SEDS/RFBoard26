// telemetry_thread.c
#include "RF-Threads.h"
#include "tx_api.h"
#include "telemetry.h"

// Stack + TCB for telemetry thread
TX_THREAD telemetry_thread;
#define TELEMETRY_THREAD_STACK_SIZE 1024u
ULONG telemetry_thread_stack[TELEMETRY_THREAD_STACK_SIZE / sizeof(ULONG)];

void telemetry_thread_entry(ULONG initial_input)
{
    (void)initial_input;

    const char started_txt[] = "Telemetry thread starting";
    log_telemetry_synchronous(SEDS_DT_MESSAGE_DATA,
                              started_txt,
                              sizeof(started_txt),
                              1);

    for (;;) {
        process_all_queues_timeout(20);
        tx_thread_sleep(10);  // 10 ticks; adjust as needed
    }
}

void create_telemetry_thread(void)
{
    UINT status = tx_thread_create(&telemetry_thread,
                                   "Telemetry Thread",
                                   telemetry_thread_entry,
                                   0,  // initial input
                                   telemetry_thread_stack,
                                   TELEMETRY_THREAD_STACK_SIZE,
                                   5,    // priority
                                   5,    // preemption threshold
                                   TX_NO_TIME_SLICE,
                                   TX_AUTO_START);

    if (status != TX_SUCCESS) {
        die("Failed to create telemetry thread: %u", (unsigned)status);
    }
}
