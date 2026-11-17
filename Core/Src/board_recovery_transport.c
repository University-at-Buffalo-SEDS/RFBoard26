#include "persistent_store_health.h"
#include "board_commissioning.h"
#include "tx_api.h"
#include "board_recovery_transport.h"
#include "board_recovery_wire.h"
#include "board_recovery_boot.h"
#include "board_recovery_floor.h"
#include "board_recovery.h"
#include "network_stamp.h"
#include <string.h>
static SedsRouter *recovery_router;
/* Delivered reliable packets refused for good and acknowledged (SEDS_OK). */
volatile uint32_t g_reliable_refusal_count __attribute__((used));
void board_recovery_transport_set_router(SedsRouter *router) { recovery_router=router; }
/* A station checked by its fixed name ("GS", "RRLS"): matches the name, an
 * address the SDK resolves to it, or, when this board never learned the
 * station (rebooted, or behind a relay before it heard the station), the
 * name's own compact identity unless another known name shares it. */
static uint8_t authorized_owner(SedsRouter *router,const SedsPacketView *packet,
                                const char *owner,size_t owner_len)
{
 return packet!=NULL && seds_router_sender_matches_station(router,
     packet->sender,packet->sender_len,owner,owner_len);
}
/* A station: the GS, or the RRLS when it takes over (name or compact alias,
 * resolved by the SDK). Freshness, epoch and nonce checks still apply. */
static uint8_t authorized_primary(SedsRouter *router,const SedsPacketView *packet)
{
 return authorized_owner(router,packet,"GS",2U) || authorized_owner(router,packet,"RRLS",4U);
}
/* Station request validity window: at most 15 s old and 1 s ahead of this
 * board's network time, judged by SEDSnet across the time steps it remembers
 * (a newer-epoch stamp passes while this board re-anchors onto it). A stamp
 * from before a forgotten step, or no network time, is never fresh. */
uint8_t board_recovery_stamp_fresh(SedsRouter*router,uint64_t key){
 bool fresh=false;if(!router)router=recovery_router;
 return router!=NULL&&key!=0U&&seds_router_network_stamp_fresh(router,network_stamp_epoch(key),network_stamp_ms(key),15000U,1000U,&fresh)==SEDS_OK&&fresh;
}
uint8_t board_recovery_command_is_stale(const SedsPacketView *packet)
{
 if (packet==0) return 1U;
 const uint64_t key=NETWORK_STAMP_OF(packet);
 /* Ordered by (epoch, stamp): a request after a backward step is newer than
  * a floor set before it; a copy stamped before the floor never is. */
 if (!board_recovery_command_time_allowed(key)) return 1U;
 if (board_recovery_floor_active() && !board_recovery_stamp_fresh(recovery_router,key)) return 1U;
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

static SedsResult commissioning_status(SedsRouter*router,uint8_t result,const BoardRecoveryRequest*request,uint32_t uptime,uint8_t extra_flags){
 BoardRecoverySnapshot snapshot={.actual_state=255};
 if(board_recovery_snapshot(&snapshot)!=0)result=BOARD_RECOVERY_RESULT_IO;
 uint64_t generation=board_commissioning_query_generation();
 if(!generation&&!board_commissioning_uncertain_first())snapshot.flags&=(uint8_t)~BOARD_RECOVERY_SUPPORTED;
 if(request->operation==BOARD_RECOVERY_RECOVER&&result==BOARD_RECOVERY_RESULT_OK&&(snapshot.flags&BOARD_RECOVERY_ABORT_LATCHED)&&!board_commissioning_pending())result=BOARD_RECOVERY_RESULT_STALE;
 uint8_t wire[BOARD_RECOVERY_STATUS_SIZE];
 board_recovery_encode_status(wire,result,snapshot.actual_state,(uint8_t)(snapshot.flags|extra_flags|BOARD_RECOVERY_VERIFIED_IMPLEMENTATION),request,generation,snapshot.abort_epoch,uptime,snapshot.actual_outputs_mask);
 return seds_router_log_queue_typed(router,SEDS_DT_BOARD_STATE_STATUS,wire,sizeof wire,1,SEDS_EK_UNSIGNED);
}
static bool commissioning_fresh(void*context,uint64_t key){return board_recovery_stamp_fresh((SedsRouter*)context,key)!=0U;}
static bool commissioning_reply(void*context,const BoardRecoveryRequest*request,uint8_t result){uint32_t uptime=(uint32_t)(((uint64_t)tx_time_get()*1000U)/TX_TIMER_TICKS_PER_SECOND);return commissioning_status((SedsRouter*)context,result,request,uptime,0U)==SEDS_OK;}
/* BOARD_STATE_STATUS is an ordered reliable stream: a status goes on air only
 * once every older one has its station receipt. The Abort push carries the
 * live snapshot, so it supersedes them; without this it waits behind them, for
 * seconds when their receipts are lost on air. Rows are read one at a time (no
 * scratch); a completed row shifts the rest down, so that index is read again.
 * The SDK never completes ABORT or another urgent type this way. */
static void retire_superseded_status(SedsRouter*router){
 SedsReliablePendingRow row;
 for(size_t index=0,guard=0;guard<64U;++guard){
  if(seds_router_copy_reliable_pending(router,index,&row,1)<=(int32_t)index)return;
  if(row.data_type==(uint32_t)SEDS_DT_BOARD_STATE_STATUS&&
     seds_router_complete_reliable(router,row.packet_id)==SEDS_OK)continue;
  ++index;
 }
}
/* Issuer of the last valid station QUERY, echoed by the Abort push. */
static uint64_t last_query_issuer;
/* Caller holds the router (FLIGHT_STATE callback): one unsolicited status for
 * an applied change to Aborted, so the station sees it even while its ordered
 * queries wait. Nonce 0, the last valid QUERY issuer, the QUERY generation. */
SedsResult board_recovery_push_abort_status(SedsRouter*router){
 if(!router)return SEDS_BAD_ARG;
 retire_superseded_status(router);
 BoardRecoveryRequest request={0};request.issuer_session=last_query_issuer;
 return commissioning_status(router,BOARD_RECOVERY_RESULT_OK,&request,
  (uint32_t)(((uint64_t)tx_time_get()*1000U)/TX_TIMER_TICKS_PER_SECOND),BOARD_RECOVERY_ABORT_LATCHED);
}
SedsResult board_recovery_handle(SedsRouter *router,const SedsPacketView *packet,
                                uint32_t local_endpoint,uint32_t uptime_ms)
{
 if (router==0 || packet==0) return SEDS_BAD_ARG;
 /* A delivered request this board will never act on (wrong type or endpoint,
  * not from the station, undecodable) is refused for good: acknowledge it so
  * the sender stops retransmitting, and count it. */
 BoardRecoveryRequest request;
 if (packet->ty!=SEDS_DT_BOARD_RECOVERY_REQUEST || packet->num_endpoints!=1U ||
     packet->endpoints==0 || packet->endpoints[0]!=local_endpoint ||
     authorized_primary(router,packet)==0U ||
     board_recovery_decode_request(packet->payload,packet->payload_len,&request)==0U)
  {++g_reliable_refusal_count;return SEDS_OK;}
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
  last_query_issuer=request.issuer_session;
  if(result==BOARD_RECOVERY_RESULT_OK){
   result=board_commissioning_query_result();
   if(board_commissioning_pending()&&!board_commissioning_note_query(&request,NETWORK_STAMP_OF(packet),snapshot.abort_epoch,router,commissioning_fresh))result=BOARD_RECOVERY_RESULT_IO;
  }
  
  return commissioning_status(router,result,&request,uptime_ms,0U);
 }
 if(result==BOARD_RECOVERY_RESULT_OK){
  int32_t staged=board_commissioning_stage(&request,NETWORK_STAMP_OF(packet),snapshot.abort_epoch,router,commissioning_reply,commissioning_fresh);
  if(staged!=-2){if(staged==-1)return SEDS_OK;return commissioning_status(router,(uint8_t)staged,&request,uptime_ms,0U);}
 }
 /* No ordinary fallback may bypass the durable transaction service. Strict
  * unclassified/tombstoned/failed metadata remains observable and inhibited. */
 return commissioning_status(router,result==BOARD_RECOVERY_RESULT_OK?BOARD_RECOVERY_RESULT_IO:result,&request,uptime_ms,0U);
}
