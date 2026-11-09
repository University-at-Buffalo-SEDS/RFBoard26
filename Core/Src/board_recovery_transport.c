#include "persistent_store_health.h"
#include "board_commissioning.h"
#include "tx_api.h"
#include "board_recovery_transport.h"
#include "board_recovery_wire.h"
#include "board_recovery_boot.h"
#include "board_recovery_floor.h"
#include "board_recovery.h"
#include <string.h>
static SedsRouter *recovery_router;
void board_recovery_transport_set_router(SedsRouter *router) { recovery_router=router; }
static uint8_t authorized_primary(SedsRouter *router,const SedsPacketView *packet)
{
 return packet!=NULL && seds_router_sender_matches_hostname(router,
     packet->sender,packet->sender_len,"GS",2U);
}
/* Station clock: the newest GS-issued timestamp (the GS QUERY; RF hosts no
 * heartbeat endpoint), advanced by the board tick since its receipt. Transit
 * only delays it, so it is a lower bound of GS time and never makes an old
 * request look fresher than the GS clock would. It only moves forward (a
 * network clock resync restarts it from the next GS packet) and is used
 * beside, never instead of, a valid SEDSnet network clock. Its writers and
 * readers all run under the telemetry router lock. */
#if TX_TIMER_TICKS_PER_SECOND != 1000
#error "The station clock counts milliseconds in ThreadX ticks"
#endif
static uint64_t station_utc;static ULONG station_tick;
static uint64_t station_estimate(void){return station_utc?station_utc+(ULONG)(tx_time_get()-station_tick):0U;}
static void station_store(uint64_t utc){station_utc=utc;station_tick=tx_time_get();}
void board_recovery_observe_station_time(SedsRouter*router,const SedsPacketView*packet){
 if(authorized_primary(router,packet)&&packet->timestamp!=UINT64_MAX&&packet->timestamp>station_estimate())station_store(packet->timestamp);
}
/* A relayed consumer cannot complete a verified SEDSnet time exchange, so a
 * wrong first anchor is only slewed at 5%. A valid network clock more than
 * 60 s from the station clock restarts timesync; the station clock restarts
 * from the next GS packet, so a GS clock step cannot repeat it every poll. */
bool board_recovery_network_clock_resync_needed(uint64_t network_now){
 uint64_t station=station_estimate();
 if(!station||network_now-station+60000U<=120000U)return false;
 station_store(0U);return true;
}
/* Freshness clock: max(valid network time, station clock); 0 when neither. */
uint64_t board_recovery_station_now(SedsRouter*router){
 uint64_t now=0U;
 if(!router||seds_router_get_network_time_ms(router,&now)!=SEDS_OK)now=0U;
 uint64_t station=station_estimate();return station>now?station:now;
}
uint8_t board_recovery_command_is_stale(const SedsPacketView *packet)
{
 if (packet==0 || !board_recovery_command_time_allowed(packet->timestamp)) return 1U;
 if (board_recovery_floor_active()) {
  uint64_t now=board_recovery_station_now(recovery_router);
  if (packet->timestamp==0U || packet->timestamp==UINT64_MAX || now==0U || now==UINT64_MAX || (now>=packet->timestamp ? now-packet->timestamp>15000U : packet->timestamp-now>1000U)) return 1U;
 }
 return 0U;
}
uint8_t board_recovery_managed_update_allowed(const SedsPacketView *packet)
{
 if (!persistent_store_controls_ready() || !board_recovery_floor_is_healthy()) return 0U;
 if (!board_recovery_floor_active()) return packet!=NULL && packet->ty==SEDS_DT_FLIGHT_STATE ? recovery_router!=NULL && authorized_primary(recovery_router,packet) : 1U;
 return recovery_router!=0 && authorized_primary(recovery_router,packet) &&
        !board_recovery_command_is_stale(packet);
}
/* A well-formed FLIGHT_STATE whose sender is not the primary station is a
 * state report (this board's own, or the FC's that the station follows),
 * never a desired-state write: callers ignore it instead of failing SDK
 * delivery or relay. */
uint8_t board_recovery_managed_report_ignorable(const SedsPacketView *packet)
{
 return packet!=NULL && packet->ty==SEDS_DT_FLIGHT_STATE && recovery_router!=NULL &&
        packet->sender!=NULL && packet->sender_len!=0U &&
        !authorized_primary(recovery_router,packet);
}
uint8_t board_recovery_automatic_is_stale(const SedsPacketView *packet)
{
 return board_recovery_command_is_stale(packet);
}

static SedsResult commissioning_status(SedsRouter*router,uint8_t result,const BoardRecoveryRequest*request,uint32_t uptime){
 BoardRecoverySnapshot snapshot={.actual_state=255};
 if(board_recovery_snapshot(&snapshot)!=0)result=BOARD_RECOVERY_RESULT_IO;
 uint64_t generation=board_commissioning_query_generation();
 if(!generation&&!board_commissioning_uncertain_first())snapshot.flags&=(uint8_t)~BOARD_RECOVERY_SUPPORTED;
 if(request->operation==BOARD_RECOVERY_RECOVER&&result==BOARD_RECOVERY_RESULT_OK&&(snapshot.flags&BOARD_RECOVERY_ABORT_LATCHED)&&!board_commissioning_pending())result=BOARD_RECOVERY_RESULT_STALE;
 uint8_t wire[BOARD_RECOVERY_STATUS_SIZE];
 board_recovery_encode_status(wire,result,snapshot.actual_state,(uint8_t)(snapshot.flags|BOARD_RECOVERY_VERIFIED_IMPLEMENTATION),request,generation,snapshot.abort_epoch,uptime,snapshot.actual_outputs_mask);
 return seds_router_log_queue_typed(router,SEDS_DT_BOARD_STATE_STATUS,wire,sizeof wire,1,SEDS_EK_UNSIGNED);
}
static uint64_t commissioning_now(void*context){return board_recovery_station_now((SedsRouter*)context);}
static bool commissioning_reply(void*context,const BoardRecoveryRequest*request,uint8_t result){uint32_t uptime=(uint32_t)(((uint64_t)tx_time_get()*1000U)/TX_TIMER_TICKS_PER_SECOND);return commissioning_status((SedsRouter*)context,result,request,uptime)==SEDS_OK;}
SedsResult board_recovery_handle(SedsRouter *router,const SedsPacketView *packet,
                                uint32_t local_endpoint,uint32_t uptime_ms)
{
 if (router==0 || packet==0 || packet->ty!=SEDS_DT_BOARD_RECOVERY_REQUEST ||
     packet->num_endpoints!=1U || packet->endpoints==0 ||
     packet->endpoints[0]!=local_endpoint) return SEDS_BAD_ARG;
 if (authorized_primary(router,packet)==0U) return SEDS_PERMISSION_DENIED;
 BoardRecoveryRequest request;
 if (board_recovery_decode_request(packet->payload,packet->payload_len,&request)==0U) return SEDS_BAD_ARG;
 BoardRecoverySnapshot snapshot={.actual_state=255U};
 uint64_t generation=board_commissioning_query_generation();
 /* Stored MAX is exhaustion; only strict eligible cache may advertiseMAX. */
 /* Eligible MAX is supplied only by the strict commissioning boot cache. */
 uint8_t result=BOARD_RECOVERY_RESULT_OK;
 if (board_recovery_snapshot(&snapshot)!=0) result=BOARD_RECOVERY_RESULT_IO;
 if ((generation==0U&&!board_commissioning_uncertain_first()) || (snapshot.flags&BOARD_RECOVERY_SUPPORTED)==0U) {
  snapshot.flags&=(uint8_t)~BOARD_RECOVERY_SUPPORTED;
  result=BOARD_RECOVERY_RESULT_UNSUPPORTED;
 }
 /* These four domains own only cache Startup, not Idle/output recovery.
  * Reject unsupported intent before loading/committing a floor or calling owner. */
 if (request.operation==BOARD_RECOVERY_RECOVER && request.desired_state!=0U)
  result=BOARD_RECOVERY_RESULT_INVALID;
 /* Commissioning has its own explicit two-leg path. QUERY only reads boot
  * caches/registers and never provisions or refreshes storage. */
 if(request.operation==BOARD_RECOVERY_QUERY){
  /* The station's own QUERY time may advance the station clock: the station
   * acts only on a reply to its nonce within its query deadline. */
  board_recovery_observe_station_time(router,packet);
  if(result==BOARD_RECOVERY_RESULT_OK){
   result=board_commissioning_query_result();
   if(board_commissioning_pending()&&!board_commissioning_note_query(&request,packet->timestamp,commissioning_now(router),snapshot.abort_epoch))result=BOARD_RECOVERY_RESULT_IO;
  }
  
  return commissioning_status(router,result,&request,uptime_ms);
 }
 if(result==BOARD_RECOVERY_RESULT_OK){
  int32_t staged=board_commissioning_stage(&request,packet->timestamp,commissioning_now(router),snapshot.abort_epoch,router,commissioning_reply,commissioning_now);
  if(staged!=-2){if(staged==-1)return SEDS_OK;return commissioning_status(router,(uint8_t)staged,&request,uptime_ms);}
 }
 /* No ordinary fallback may bypass the durable transaction service. Strict
  * unclassified/tombstoned/failed metadata remains observable and inhibited. */
 return commissioning_status(router,result==BOARD_RECOVERY_RESULT_OK?BOARD_RECOVERY_RESULT_IO:result,&request,uptime_ms);
}
