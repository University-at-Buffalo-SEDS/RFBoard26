#ifndef BOARD_RECOVERY_TRANSPORT_H
#define BOARD_RECOVERY_TRANSPORT_H
#include "sedsnet.h"
#include "sedsnet_config.h"
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
/* Advance the station clock from an authorized GS packet's timestamp. */
void board_recovery_observe_station_time(SedsRouter*router,const SedsPacketView*packet);
/* Freshness clock: max(valid network time, station clock), or the station
 * clock alone after it followed a GS step back until they agree; 0 when neither. */
uint64_t board_recovery_station_now(SedsRouter*router);
/* A mark (floor, last applied stamp) more than this far ahead of the station
 * clock predates a backward GS clock step. */
#define BOARD_RECOVERY_STATION_STEP_BACK_MS UINT64_C(60000)
uint8_t board_recovery_station_behind(uint64_t mark,uint64_t station);
/* Nonzero after the station clock followed a GS step back, until the network
 * clock agrees with it again. */
uint8_t board_recovery_station_stepped_back(void);
/* Network thread only: a valid network clock more than 60 s from the station
 * clock (which then restarts from the next GS packet). */
bool board_recovery_network_clock_resync_needed(uint64_t network_now);
#endif
