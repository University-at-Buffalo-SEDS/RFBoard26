#ifndef BOARD_RECOVERY_TRANSPORT_H
#define BOARD_RECOVERY_TRANSPORT_H
#include "sedsnet.h"
#include "sedsnet_config.h"
#include <stdbool.h>
void board_recovery_transport_set_router(SedsRouter *router);
/* Delivered reliable packets refused for good, yet acknowledged. */
extern volatile uint32_t g_reliable_refusal_count;
/* Caller holds the router: one nonce-0 Abort-latched BOARD_STATE_STATUS. */
SedsResult board_recovery_push_abort_status(SedsRouter *router);
SedsResult board_recovery_handle(SedsRouter *router,const SedsPacketView *packet,
                                uint32_t local_endpoint,uint32_t uptime_ms);
/* Application replay floor; SDK stays generic. No current-time/RNG claim. */
uint8_t board_recovery_command_is_stale(const SedsPacketView *packet);
uint8_t board_recovery_managed_update_allowed(const SedsPacketView *packet);
uint8_t board_recovery_managed_report_ignorable(const SedsPacketView *packet);
uint8_t board_recovery_automatic_is_stale(const SedsPacketView *packet);
/* A station stamp (network_stamp.h key) at most 15 s old and 1 s ahead of
 * the network clock, across remembered time steps; never without network time. */
uint8_t board_recovery_stamp_fresh(SedsRouter*router,uint64_t key);
#endif
