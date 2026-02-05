// neom9n_thread.c
#include "RF-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "neom9n.h"

// Stack + TCB for neom9n thread
TX_THREAD neom9n_thread;
#define NEOM9N_THREAD_STACK_SIZE 1024u
ULONG neom9n_thread_stack[NEOM9N_THREAD_STACK_SIZE / sizeof(ULONG)];

void neom9n_thread_entry(ULONG initial_input) 
{
    (void)initial_input;

    const char started_txt[] = "NEOM9N thread starting";
    log_telemetry_asynchronous(SEDS_DT_MESSAGE_DATA, started_txt, sizeof(started_txt), 1); //inital log statement

    for (;;) {
        __NOP();    // Replace with thread function
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
