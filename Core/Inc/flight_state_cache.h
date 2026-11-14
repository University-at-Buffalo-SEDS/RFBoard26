#ifndef FLIGHT_STATE_CACHE_H
#define FLIGHT_STATE_CACHE_H

#ifdef TELEMETRY_ENABLED
#include "sedsnet.h"
#endif
#include <stdbool.h>
#include <stdint.h>

/* FLIGHT_STATE contract (a): begin */
/* Network FLIGHT_STATE, contract set (a): the only values ever on the wire.
 * Fill covers all fill activity (the fill sub-phases live in the station);
 * InFlight covers the FC's Ascent through Reefing. */
#define FLIGHT_STATE_NET_STARTUP 0U
#define FLIGHT_STATE_NET_IDLE 1U
#define FLIGHT_STATE_NET_FILL 2U
#define FLIGHT_STATE_NET_ARMED 6U
#define FLIGHT_STATE_NET_LAUNCH 7U
#define FLIGHT_STATE_NET_LANDED 13U
#define FLIGHT_STATE_NET_ABORTED 15U
#define FLIGHT_STATE_NET_IN_FLIGHT 16U
#define FLIGHT_STATE_NET_MAX 16U
#define FLIGHT_STATE_NET_INVALID 0xFFU

/* A received value on set (a). Values retired from the wire (older firmware
 * on a mixed bench) map to the nearest network value and set *retired:
 * 3..5 -> Fill, 8..12 -> InFlight, 14 -> Landed. Above InFlight is INVALID. */
static inline uint8_t flight_state_from_wire(uint8_t raw, uint8_t *retired)
{
    uint8_t state = raw;
    uint8_t mapped = 1U;
    if (raw >= 3U && raw <= 5U) state = FLIGHT_STATE_NET_FILL;
    else if (raw >= 8U && raw <= 12U) state = FLIGHT_STATE_NET_IN_FLIGHT;
    else if (raw == 14U) state = FLIGHT_STATE_NET_LANDED;
    else {
        mapped = 0U;
        if (raw > FLIGHT_STATE_NET_MAX) state = FLIGHT_STATE_NET_INVALID;
    }
    if (retired != 0) *retired = mapped;
    return state;
}
/* FLIGHT_STATE contract (a): end */


void flight_state_cache_restore(void);
bool flight_state_cache_has_value(void);
uint8_t flight_state_cache_value(void);
/* A received FLIGHT_STATE on contract set (a); counts and logs (once) a
 * retired value. FLIGHT_STATE_NET_INVALID above InFlight. */
uint8_t flight_state_cache_from_wire(uint8_t raw);
/* Explicit operator reset only. Persist and verify Startup before reporting
 * success; never alter power rails, outputs, acquisition or network identity. */
int32_t flight_state_cache_reset_startup(void);
#ifdef TELEMETRY_ENABLED
SedsResult flight_state_cache_init(SedsRouter *router);
SedsResult flight_state_cache_poll(SedsRouter *router);
SedsResult flight_state_cache_apply_network_update(const SedsPacketView *packet);
/* True when a station FLIGHT_STATE write is older than (or repeats) the last
 * one applied from its sender; remember=true records an applied write. */
bool flight_state_cache_order(const SedsPacketView *packet, bool remember);
#endif

#endif
