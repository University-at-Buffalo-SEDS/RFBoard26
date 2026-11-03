#include "flight_state_cache.h"
#include "main.h"
#include "sedsnet_config.h"
#ifdef TELEMETRY_ENABLED
#include "board_recovery_transport.h"
#include "ota_stream.h"
#endif

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
#ifdef TELEMETRY_ENABLED
static bool g_network_value_seen;
/* UINT8_MAX cannot be a valid state; volatile application is not flash durability. */
static uint8_t g_persisted_state = UINT8_MAX;
static uint32_t g_last_refresh_ms;
static SedsRouter *g_cache_router;
static SedsResult persist_update(const SedsPacketView *packet, void *user);
/* SDK 4.3.6 caches inbound data before its callback. Retire only this
 * observation; keep read-only permissions, callbacks and advertisements. */
static SedsResult clear_managed_cache(void)
{
    if (g_cache_router == NULL) return SEDS_BAD_ARG;
    return seds_router_invalidate_network_variable_cache(
        g_cache_router, SEDS_DT_FLIGHT_STATE, NULL);
}
#endif

#ifdef TELEMETRY_ENABLED
__attribute__((weak)) SedsResult
flight_state_cache_apply_network_update(const SedsPacketView *packet)
{
    (void)packet;
    return SEDS_OK;
}

#endif

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
#ifdef TELEMETRY_ENABLED
    g_persisted_state = state;
#endif
#ifdef TELEMETRY_ENABLED
    /* A restored unsafe phase is evidence against automatic OTA, not proof
     * that this board physically launched or owns any outputs. */
    if (ota_stream_state_begin(state)) ota_stream_state_end();
#endif
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

int32_t flight_state_cache_reset_startup(void)
{
    const uint8_t startup = 0U;
    uint8_t verified = UINT8_MAX;
    size_t size = sizeof(verified);
    /* A failed boot-time storage initialization may be explicitly retried by
     * this manual operation. No background wear/retry loop is introduced. */
    if (persistent_store_init() != LAUNCHCORE_PERSIST_OK ||
        persistent_store_set(FLIGHT_STATE_PERSIST_KEY, &startup, sizeof(startup)) != LAUNCHCORE_PERSIST_OK ||
        persistent_store_get(FLIGHT_STATE_PERSIST_KEY, &verified, &size) != LAUNCHCORE_PERSIST_OK ||
        size != sizeof(verified) || verified != startup) {
        ++g_flight_state_cache_errors;
        return -9;
    }
    g_persist_ready = true;
    ++g_flight_state_cache_writes;
#ifdef TELEMETRY_ENABLED
    if (clear_managed_cache() != SEDS_OK) {
        ++g_flight_state_cache_errors;
        return -9;
    }
    g_persisted_state = startup;
    g_network_value_seen = false;
    g_last_refresh_ms = HAL_GetTick();
#endif
    g_flight_state_cache_value = startup;
    g_has_value = true;
    return 0;
}

#ifdef TELEMETRY_ENABLED
static SedsResult persist_update(const SedsPacketView *packet, void *user)
{
    (void)user;
    if (packet == NULL || packet->ty != SEDS_DT_FLIGHT_STATE ||
        packet->payload == NULL || packet->payload_len != 1U ||
        packet->payload[0] > FLIGHT_STATE_MAX_VALUE)
    {
        /* Public SDK435 caches managed data before invoking this callback.
         * A malformed FSTATE must not poison future ordering/cache reads. */
        if (packet != NULL && packet->ty == SEDS_DT_FLIGHT_STATE)
            (void)clear_managed_cache();
        g_flight_state_cache_errors++;
        return SEDS_HANDLER_ERROR;
    }

    if (!board_recovery_managed_update_allowed(packet)) {
        (void)clear_managed_cache();
        ++g_flight_state_cache_errors;
        return SEDS_HANDLER_ERROR;
    }
    const uint8_t state = packet->payload[0];
    if (!ota_stream_state_begin(state)) {
        (void)clear_managed_cache();
        ++g_flight_state_cache_errors;
        return SEDS_HANDLER_ERROR;
    }
    const SedsResult apply_result = flight_state_cache_apply_network_update(packet);
    if (apply_result != SEDS_OK) {
        ota_stream_state_end();
        (void)clear_managed_cache();
        g_flight_state_cache_errors++;
        return apply_result;
    }
    const bool changed = state != g_persisted_state;
    g_has_value = true;
    g_network_value_seen = true;
    g_flight_state_cache_value = state;
    g_flight_state_cache_updates++;
    if (!changed) { ota_stream_state_end(); return SEDS_OK; }

    if (!g_persist_ready ||
        persistent_store_set(FLIGHT_STATE_PERSIST_KEY, &state,
                             sizeof(state)) != LAUNCHCORE_PERSIST_OK)
    {
        /* Preserve the existing volatile apply-before-persist policy, but
         * retire the SDK cache so an identical reliable retry re-enters this
         * callback instead of being accepted without retrying durability.
         * g_network_value_seen is unchanged: no extra polling traffic. */
        (void)clear_managed_cache();
        g_flight_state_cache_errors++;
        ota_stream_state_end();
        return SEDS_HANDLER_ERROR;
    }
    g_persisted_state = state;
    g_flight_state_cache_writes++;
    ota_stream_state_end();
    return SEDS_OK;
}

SedsResult flight_state_cache_init(SedsRouter *router)
{
    if (router == NULL) return SEDS_BAD_ARG;
    /* A valid reinitialization must establish both services before cache
     * retirement may report success. Invalid NULL input preserves readiness. */
    g_cache_router = NULL;
    flight_state_cache_restore();
    SedsResult result = seds_router_enable_network_variable(
        router, SEDS_DT_FLIGHT_STATE, true, false);
    if (result != SEDS_OK) { ++g_flight_state_cache_errors; return result; }
    result = seds_router_on_network_variable_update(
        router, SEDS_DT_FLIGHT_STATE, persist_update, NULL);
    if (result != SEDS_OK) { ++g_flight_state_cache_errors; return result; }
    g_cache_router = router;
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

#endif /* TELEMETRY_ENABLED */
