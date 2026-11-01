#ifndef BOARD_RECOVERY_TRANSPORT_H
#define BOARD_RECOVERY_TRANSPORT_H
#include "sedsnet.h"
#include "sedsnet_config.h"
void board_recovery_transport_set_router(SedsRouter *router);
SedsResult board_recovery_handle(SedsRouter *router,const SedsPacketView *packet,
                                uint32_t local_endpoint,uint32_t uptime_ms);
/* Application replay floor; SDK stays generic. No current-time/RNG claim. */
uint8_t board_recovery_command_is_stale(const SedsPacketView *packet);
uint8_t board_recovery_managed_update_allowed(const SedsPacketView *packet);
uint8_t board_recovery_automatic_is_stale(const SedsPacketView *packet);
#endif
