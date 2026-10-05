#pragma once
#include "sedsnet_config.h"

/* Call under the telemetry lock, before constructing any router. A router
 * retry must reuse the same arena; replacing it can double the pool demand. */
#ifdef SEDS_ENABLE_COMPACT_PACKET_STORE
volatile int32_t g_board_packet_store_init_result = SEDS_IO;
static unsigned g_board_packet_store_ready;
#endif
static inline SedsResult board_packet_store_init(void)
{
#ifdef SEDS_ENABLE_COMPACT_PACKET_STORE
    if (g_board_packet_store_ready) return SEDS_OK;
    g_board_packet_store_init_result = seds_packet_store_configure(
        BOARD_PACKET_ARENA_BYTES, BOARD_PACKET_ARENA_HANDLES, 512U);
    if (g_board_packet_store_init_result != SEDS_OK)
        return (SedsResult)g_board_packet_store_init_result;
    g_board_packet_store_ready = 1U;
#endif
    return SEDS_OK;
}
