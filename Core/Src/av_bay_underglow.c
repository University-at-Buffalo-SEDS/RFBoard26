#include "av_bay_underglow.h"
#include "main.h"

#include "persistent_store.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

extern volatile uint32_t g_telemetry_discovery_seen;

#define UNDERGLOW_PERSIST_KEY 0x55474C57u
#define NETWORK_VARIABLE_UNSYNCED_RETRY_MS 500U
/* Underglow is cosmetic: it lives in RAM and reaches flash only once a new
 * value has held for UNDERGLOW_PERSIST_SETTLE_MS, and at most once per
 * UNDERGLOW_PERSIST_MIN_INTERVAL_MS. A toggle never writes flash itself. */
#ifndef UNDERGLOW_PERSIST_SETTLE_MS
#define UNDERGLOW_PERSIST_SETTLE_MS 2000U
#endif
#ifndef UNDERGLOW_PERSIST_MIN_INTERVAL_MS
#define UNDERGLOW_PERSIST_MIN_INTERVAL_MS 60000U
#endif

volatile uint32_t g_av_bay_underglow_enabled = 0U;
volatile uint32_t g_av_bay_underglow_updates = 0U;
volatile uint32_t g_av_bay_underglow_persist_restores = 0U;
volatile uint32_t g_av_bay_underglow_persist_writes = 0U;
volatile uint32_t g_av_bay_underglow_persist_errors = 0U;

static bool g_persist_ready = false;
/* The value durable in flash; UINT8_MAX until one is read or written. */
static uint8_t g_persisted = UINT8_MAX;
#ifdef TELEMETRY_ENABLED
static bool g_persist_pending = false;
static bool g_persist_attempted = false;
static uint32_t g_changed_ms = 0U;
static uint32_t g_attempt_ms = 0U;
#endif
static bool g_restore_attempted = false;
#ifdef TELEMETRY_ENABLED
static bool g_network_value_seen = false;
static uint32_t g_last_refresh_ms = 0U;
#endif

static void drive_underglow(bool enabled)
{
    g_av_bay_underglow_enabled = enabled ? 1U : 0U;
    HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void av_bay_underglow_restore(void)
{
    if (g_restore_attempted) return;
    g_restore_attempted = true;

    uint8_t enabled = 0U;
    size_t enabled_size = sizeof(enabled);

    const launchcore_persist_status_t init_status = persistent_store_init();
    if (init_status != LAUNCHCORE_PERSIST_OK)
    {
        g_av_bay_underglow_persist_errors++;
        return;
    }
    g_persist_ready = true;

    const launchcore_persist_status_t status = persistent_store_get(
        UNDERGLOW_PERSIST_KEY, &enabled, &enabled_size);
    if (status == LAUNCHCORE_PERSIST_NOT_FOUND) return;
    if (status != LAUNCHCORE_PERSIST_OK || enabled_size != sizeof(enabled) ||
        enabled > 1U)
    {
        g_av_bay_underglow_persist_errors++;
        return;
    }

    g_persisted = enabled;
    g_av_bay_underglow_persist_restores++;
    drive_underglow(enabled != 0U);
}

#ifdef TELEMETRY_ENABLED
static SedsResult apply_underglow(const SedsPacketView *packet, void *user)
{
    (void)user;
    if (packet == NULL || packet->ty != SEDS_DT_AV_BAY_UNDERGLOW ||
        packet->payload == NULL || packet->payload_len != 1U)
    {
        return SEDS_HANDLER_ERROR;
    }

    const bool enabled = packet->payload[0] != 0U;
    if (g_av_bay_underglow_enabled != (uint32_t)enabled) g_changed_ms = HAL_GetTick();
    drive_underglow(enabled);
    g_network_value_seen = true;
    g_av_bay_underglow_updates++;
    /* Acknowledged now; flash follows from the poll once the value settles. */
    g_persist_pending = (uint8_t)enabled != g_persisted;
    return SEDS_OK;
}

/* A failed write is retried at the same rate, never by withholding a receipt. */
static void persist_settled(void)
{
    const uint32_t now_ms = HAL_GetTick();
    if (!g_persist_pending ||
        (uint32_t)(now_ms - g_changed_ms) < UNDERGLOW_PERSIST_SETTLE_MS ||
        (g_persist_attempted &&
         (uint32_t)(now_ms - g_attempt_ms) < UNDERGLOW_PERSIST_MIN_INTERVAL_MS)) return;
    g_persist_attempted = true;
    g_attempt_ms = now_ms;
    const uint8_t stored = g_av_bay_underglow_enabled != 0U ? 1U : 0U;
    if (!g_persist_ready ||
        persistent_store_set(UNDERGLOW_PERSIST_KEY, &stored, sizeof(stored)) != LAUNCHCORE_PERSIST_OK)
    {
        g_av_bay_underglow_persist_errors++;
        return;
    }
    g_persisted = stored;
    g_persist_pending = false;
    g_av_bay_underglow_persist_writes++;
}

SedsResult av_bay_underglow_init(SedsRouter *router)
{
    if (router == NULL) return SEDS_BAD_ARG;
    av_bay_underglow_restore();
    SedsResult result = seds_router_enable_network_variable(
        router, SEDS_DT_AV_BAY_UNDERGLOW, true, false);
    if (result != SEDS_OK) return result;
    result = seds_router_on_network_variable_update(
        router, SEDS_DT_AV_BAY_UNDERGLOW, apply_underglow, NULL);
    if (result != SEDS_OK) return result;
    /* Never wait for a reply on the router's own service thread.  The cached
     * flash value remains active until the authoritative network value arrives. */
    g_last_refresh_ms = HAL_GetTick();
    return SEDS_OK;
}

SedsResult av_bay_underglow_poll(SedsRouter *router)
{
    if (router == NULL) return SEDS_BAD_ARG;
    persist_settled();
    if (g_network_value_seen) return SEDS_OK;
    if (g_telemetry_discovery_seen == 0U) return SEDS_OK;
    const uint32_t now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - g_last_refresh_ms) <
        NETWORK_VARIABLE_UNSYNCED_RETRY_MS) return SEDS_OK;
    g_last_refresh_ms = now_ms;
    return seds_router_request_managed_variable(
        router, SEDS_DT_AV_BAY_UNDERGLOW);
}

#endif
