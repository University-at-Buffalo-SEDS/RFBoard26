#include "flight_state_cache.h"
#include "main.h"
#include "sedsnet_config.h"

#include "persistent_store.h"

#include <stddef.h>
#include <string.h>

extern volatile uint32_t g_telemetry_discovery_seen;

#define FLIGHT_STATE_PERSIST_KEY 0x46535445u
#define FLIGHT_STATE_MAX_VALUE 15U
#define FLIGHT_STATE_UNSYNCED_RETRY_MS 500U

volatile uint32_t g_flight_state_cache_value __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_flight_state_cache_restores __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_flight_state_cache_writes __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_flight_state_cache_updates __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_flight_state_cache_errors __attribute__((used, externally_visible)) = 0U;

static bool g_restore_attempted;
static bool g_persist_ready;
static bool g_has_value;
static bool g_network_value_seen;
static uint32_t g_last_refresh_ms;

__attribute__((weak)) SedsResult
flight_state_cache_apply_network_update(const SedsPacketView *packet)
{
    (void)packet;
    return SEDS_OK;
}

void flight_state_cache_restore(void)
{
    if (g_restore_attempted) return;
    g_restore_attempted = true;

    if (persistent_store_init() != LAUNCHCORE_PERSIST_OK)
    {
        g_flight_state_cache_errors++;
        return;
    }
    g_persist_ready = true;

    uint8_t state = 0U;
    size_t size = sizeof(state);
    const launchcore_persist_status_t status = persistent_store_get(
        FLIGHT_STATE_PERSIST_KEY, &state, &size);
    if (status == LAUNCHCORE_PERSIST_NOT_FOUND) return;
    if (status != LAUNCHCORE_PERSIST_OK || size != sizeof(state) ||
        state > FLIGHT_STATE_MAX_VALUE)
    {
        g_flight_state_cache_errors++;
        return;
    }

    g_has_value = true;
    g_flight_state_cache_value = state;
    g_flight_state_cache_restores++;
}

bool flight_state_cache_has_value(void)
{
    return g_has_value;
}

uint8_t flight_state_cache_value(void)
{
    return (uint8_t)g_flight_state_cache_value;
}

static SedsResult persist_update(const SedsPacketView *packet, void *user)
{
    (void)user;
    if (packet == NULL || packet->ty != SEDS_DT_FLIGHT_STATE ||
        packet->payload == NULL || packet->payload_len != 1U ||
        packet->payload[0] > FLIGHT_STATE_MAX_VALUE)
    {
        g_flight_state_cache_errors++;
        return SEDS_HANDLER_ERROR;
    }

    const uint8_t state = packet->payload[0];
    const SedsResult apply_result = flight_state_cache_apply_network_update(packet);
    if (apply_result != SEDS_OK) { g_flight_state_cache_errors++; return apply_result; }
    const bool changed = !g_has_value ||
                         state != (uint8_t)g_flight_state_cache_value;
    g_has_value = true;
    g_network_value_seen = true;
    g_flight_state_cache_value = state;
    g_flight_state_cache_updates++;
    if (!changed) return SEDS_OK;

    if (!g_persist_ready ||
        persistent_store_set(FLIGHT_STATE_PERSIST_KEY, &state,
                             sizeof(state)) != LAUNCHCORE_PERSIST_OK)
    {
        g_flight_state_cache_errors++;
        return SEDS_HANDLER_ERROR;
    }
    g_flight_state_cache_writes++;
    return SEDS_OK;
}

SedsResult flight_state_cache_init(SedsRouter *router)
{
    if (router == NULL) return SEDS_BAD_ARG;
    flight_state_cache_restore();
    SedsResult result = seds_router_enable_network_variable(
        router, SEDS_DT_FLIGHT_STATE, true, false);
    if (result != SEDS_OK) return result;
    result = seds_router_on_network_variable_update(
        router, SEDS_DT_FLIGHT_STATE, persist_update, NULL);
    if (result != SEDS_OK) return result;
    g_last_refresh_ms = HAL_GetTick();
    return SEDS_OK;
}

SedsResult flight_state_cache_poll(SedsRouter *router)
{
    if (router == NULL) return SEDS_BAD_ARG;
    if (g_network_value_seen) return SEDS_OK;
    if (g_telemetry_discovery_seen == 0U) return SEDS_OK;
    const uint32_t now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - g_last_refresh_ms) <
        FLIGHT_STATE_UNSYNCED_RETRY_MS) return SEDS_OK;
    g_last_refresh_ms = now_ms;
    return seds_router_request_managed_variable(router, SEDS_DT_FLIGHT_STATE);
}
