#include "flight_state_cache.h"
#include "main.h"
#include "sedsnet_config.h"

#include "persistent_store.h"

#include <stddef.h>
#include <string.h>

#define FLIGHT_STATE_PERSIST_KEY 0x46535445u
#define FLIGHT_STATE_MAX_VALUE 15U
#define FLIGHT_STATE_REFRESH_INTERVAL_MS 250U
#define FLIGHT_STATE_PACKED_CAPACITY 128U

volatile uint32_t g_flight_state_cache_value __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_flight_state_cache_restores __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_flight_state_cache_writes __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_flight_state_cache_updates __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_flight_state_cache_errors __attribute__((used, externally_visible)) = 0U;

static bool g_restore_attempted;
static bool g_persist_ready;
static bool g_has_value;
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

static SedsResult seed_cached_value(SedsRouter *router)
{
    if (!g_has_value) return SEDS_OK;
    const uint32_t endpoint = (uint32_t)SEDS_EP_FLIGHT_STATE;
    const uint8_t state = (uint8_t)g_flight_state_cache_value;
    const SedsPacketView view = {
        .ty = (uint32_t)SEDS_DT_FLIGHT_STATE,
        .data_size = sizeof(state),
        .sender = "LOCAL_CACHE",
        .sender_len = sizeof("LOCAL_CACHE") - 1U,
        .endpoints = &endpoint,
        .num_endpoints = 1U,
        .timestamp = 0U,
        .payload = &state,
        .payload_len = sizeof(state),
    };
    uint8_t packed[FLIGHT_STATE_PACKED_CAPACITY];
    const int32_t packed_len = seds_pkt_pack_len(&view);
    if (packed_len <= 0 || (size_t)packed_len > sizeof(packed) ||
        seds_pkt_pack(&view, packed, sizeof(packed)) != packed_len)
    {
        return SEDS_HANDLER_ERROR;
    }
    return seds_router_seed_managed_variable_packed(
        router, packed, (size_t)packed_len);
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
    result = seed_cached_value(router);
    if (result != SEDS_OK) return result;
    return flight_state_cache_poll(router);
}

SedsResult flight_state_cache_poll(SedsRouter *router)
{
    if (router == NULL) return SEDS_BAD_ARG;
    const uint32_t now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - g_last_refresh_ms) <
        FLIGHT_STATE_REFRESH_INTERVAL_MS) return SEDS_OK;
    g_last_refresh_ms = now_ms;
    const int32_t result = seds_router_get_network_variable_packed_len(
        router, SEDS_DT_FLIGHT_STATE, 1000U);
    return result < 0 ? (SedsResult)result : SEDS_OK;
}
