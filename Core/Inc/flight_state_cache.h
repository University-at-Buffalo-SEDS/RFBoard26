#ifndef FLIGHT_STATE_CACHE_H
#define FLIGHT_STATE_CACHE_H

#ifdef TELEMETRY_ENABLED
#include "sedsnet.h"
#endif
#include <stdbool.h>
#include <stdint.h>

void flight_state_cache_restore(void);
bool flight_state_cache_has_value(void);
uint8_t flight_state_cache_value(void);
/* Explicit operator reset only. Persist and verify Startup before reporting
 * success; never alter power rails, outputs, acquisition or network identity. */
int32_t flight_state_cache_reset_startup(void);
#ifdef TELEMETRY_ENABLED
SedsResult flight_state_cache_init(SedsRouter *router);
SedsResult flight_state_cache_poll(SedsRouter *router);
SedsResult flight_state_cache_apply_network_update(const SedsPacketView *packet);
#endif

#endif
