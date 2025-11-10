#pragma once
#include "tx_api.h"

/* ------ Telemetry Thread ------ */
extern TX_THREAD telemetry_thread;
extern ULONG telemetry_thread_stack[];

void telemetry_thread_entry(ULONG initial_input);
void create_telemetry_thread(void);
/* ------ Telemetry Thread ------ */