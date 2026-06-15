#pragma once
#include "tx_api.h"
#include <stdint.h>

/* ------ Telemetry Thread ------ */
extern TX_THREAD telemetry_thread;

void telemetry_thread_entry(ULONG initial_input);
UINT create_telemetry_thread(TX_BYTE_POOL *byte_pool);
/* ------ Telemetry Thread ------ */


/* ------- NEOM9N Thread -------- */
extern TX_THREAD neom9n_thread;
extern volatile uint8_t g_neom9n_has_fix;

void neom9n_thread_entry(ULONG initial_input);
UINT create_neom9n_thread(TX_BYTE_POOL *byte_pool);
/* ------- NEOM9N Thread -------- */
