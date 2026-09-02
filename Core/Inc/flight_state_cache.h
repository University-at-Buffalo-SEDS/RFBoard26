#ifndef FLIGHT_STATE_CACHE_H
#define FLIGHT_STATE_CACHE_H

#include "sedsnet.h"
#include <stdbool.h>
#include <stdint.h>

void flight_state_cache_restore(void);
bool flight_state_cache_has_value(void);
uint8_t flight_state_cache_value(void);
SedsResult flight_state_cache_init(SedsRouter *router);
SedsResult flight_state_cache_poll(SedsRouter *router);
SedsResult flight_state_cache_apply_network_update(const SedsPacketView *packet);

#endif
