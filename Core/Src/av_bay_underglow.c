#include "av_bay_underglow.h"
#include "main.h"

#include "persistent_store.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UNDERGLOW_PERSIST_KEY 0x55474C57u
#define NETWORK_VARIABLE_REFRESH_INTERVAL_MS 1000U

volatile uint32_t g_av_bay_underglow_enabled = 0U;
volatile uint32_t g_av_bay_underglow_updates = 0U;
volatile uint32_t g_av_bay_underglow_persist_restores = 0U;
volatile uint32_t g_av_bay_underglow_persist_writes = 0U;
volatile uint32_t g_av_bay_underglow_persist_errors = 0U;

static bool g_persist_ready = false;
static bool g_persist_has_value = false;
static bool g_restore_attempted = false;

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

    g_persist_has_value = true;
    g_av_bay_underglow_persist_restores++;
    drive_underglow(enabled != 0U);
}

static SedsResult apply_underglow(const SedsPacketView *packet, void *user)
{
    (void)user;
    if (packet == NULL || packet->ty != SEDS_DT_AV_BAY_UNDERGLOW ||
        packet->payload == NULL || packet->payload_len != 1U)
    {
        return SEDS_HANDLER_ERROR;
    }

    const bool enabled = packet->payload[0] != 0U;
    const bool needs_persist = !g_persist_has_value ||
                               g_av_bay_underglow_enabled != (uint32_t)enabled;
    drive_underglow(enabled);
    g_av_bay_underglow_updates++;

    if (needs_persist)
    {
        const uint8_t stored = enabled ? 1U : 0U;
        const launchcore_persist_status_t status = g_persist_ready
            ? persistent_store_set(UNDERGLOW_PERSIST_KEY, &stored,
                                   sizeof(stored))
            : LAUNCHCORE_PERSIST_STORAGE_ERROR;
        if (status != LAUNCHCORE_PERSIST_OK)
        {
            g_av_bay_underglow_persist_errors++;
            return SEDS_HANDLER_ERROR;
        }
        g_persist_has_value = true;
        g_av_bay_underglow_persist_writes++;
    }
    return SEDS_OK;
}

SedsResult av_bay_underglow_init(SedsRouter *router)
{
    if (router == NULL) return SEDS_BAD_ARG;
    av_bay_underglow_restore();
    SedsResult result = seds_router_enable_network_variable(
        router, SEDS_DT_AV_BAY_UNDERGLOW, true, false);
    if (result != SEDS_OK) return result;
    return seds_router_on_network_variable_update(
        router, SEDS_DT_AV_BAY_UNDERGLOW, apply_underglow, NULL);
}

SedsResult av_bay_underglow_poll(SedsRouter *router)
{
    static uint32_t last_refresh_ms = 0U;
    if (router == NULL) return SEDS_BAD_ARG;
    const uint32_t now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - last_refresh_ms) <
        NETWORK_VARIABLE_REFRESH_INTERVAL_MS) return SEDS_OK;
    last_refresh_ms = now_ms;
    const int32_t result = seds_router_get_network_variable_packed_len(
        router, SEDS_DT_AV_BAY_UNDERGLOW, 5000U);
    return result < 0 ? (SedsResult)result : SEDS_OK;
}
