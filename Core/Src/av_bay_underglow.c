#include "av_bay_underglow.h"
#include "main.h"

volatile uint32_t g_av_bay_underglow_enabled = 0U;
volatile uint32_t g_av_bay_underglow_updates = 0U;

static SedsResult apply_underglow(const SedsPacketView *packet, void *user)
{
    (void)user;
    if (packet == NULL || packet->ty != SEDS_DT_AV_BAY_UNDERGLOW ||
        packet->payload == NULL || packet->payload_len != 1U)
    {
        return SEDS_HANDLER_ERROR;
    }
    g_av_bay_underglow_enabled = packet->payload[0] != 0U ? 1U : 0U;
    g_av_bay_underglow_updates++;
    HAL_GPIO_WritePin(BLUE_LEDS_GPIO_Port, BLUE_LEDS_Pin,
                      g_av_bay_underglow_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return SEDS_OK;
}

SedsResult av_bay_underglow_init(SedsRouter *router)
{
    if (router == NULL) return SEDS_BAD_ARG;
    SedsResult result = seds_router_enable_network_variable(
        router, SEDS_DT_AV_BAY_UNDERGLOW, true, false);
    if (result != SEDS_OK) return result;
    result = seds_router_on_network_variable_update(
        router, SEDS_DT_AV_BAY_UNDERGLOW, apply_underglow, NULL);
    if (result != SEDS_OK) return result;
    return SEDS_OK;
}

SedsResult av_bay_underglow_poll(SedsRouter *router)
{
    if (router == NULL) return SEDS_BAD_ARG;
    int32_t result = seds_router_get_network_variable_packed_len(
        router, SEDS_DT_AV_BAY_UNDERGLOW, 5000U);
    return result < 0 ? (SedsResult)result : SEDS_OK;
}
