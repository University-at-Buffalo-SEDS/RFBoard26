#include "board_recovery_transport.h"
#include "board_recovery_wire.h"
#include "board_recovery_boot.h"
#include "board_recovery_floor.h"
#include "board_recovery.h"
#include <string.h>
static SedsRouter *recovery_router;
void board_recovery_transport_set_router(SedsRouter *router) { recovery_router=router; }
static uint8_t previous_request[BOARD_RECOVERY_REQUEST_SIZE];
static uint8_t previous_request_valid;
static uint8_t authorized_primary(SedsRouter *router,const SedsPacketView *packet)
{
 if (packet->sender==0) return 0U;
 if (packet->sender_len==2U && memcmp(packet->sender,"GS",2U)==0) return 1U;
 if (packet->sender_len<7U || packet->sender_len>16U ||
     memcmp(packet->sender,"@addr:",6U)!=0) return 0U;
 uint32_t address=0U;
 for (size_t i=6U;i<packet->sender_len;i++) {
  unsigned digit=(unsigned char)packet->sender[i]-(unsigned)'0';
  if (digit>9U || address>(UINT32_MAX-digit)/10U) return 0U;
  address=address*10U+digit;
 }
 uint32_t primary=0U;
 return address!=0U && seds_router_resolve_hostname_address(router,"GS",2U,&primary)==SEDS_OK &&
        primary!=0U && address==primary;
}
uint8_t board_recovery_command_is_stale(const SedsPacketView *packet)
{
 if (packet==0 || !board_recovery_command_time_allowed(packet->timestamp)) return 1U;
 if (board_recovery_floor_active()) {
  uint64_t now=0U;
  if (recovery_router==0 || seds_router_get_network_time_ms(recovery_router,&now)!=SEDS_OK ||
      packet->timestamp==0U || (now>=packet->timestamp ? now-packet->timestamp>5000U : packet->timestamp-now>1000U)) return 1U;
 }
 return 0U;
}
uint8_t board_recovery_managed_update_allowed(const SedsPacketView *packet)
{
 if (!board_recovery_floor_is_healthy()) return 0U;
 if (!board_recovery_floor_active()) return 1U;
 return recovery_router!=0 && authorized_primary(recovery_router,packet) &&
        !board_recovery_command_is_stale(packet);
}
uint8_t board_recovery_automatic_is_stale(const SedsPacketView *packet)
{
 return board_recovery_command_is_stale(packet);
}
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
 uint64_t generation=request.operation==BOARD_RECOVERY_QUERY ?
     board_recovery_boot_generation() : board_recovery_boot_current();
 uint8_t result=BOARD_RECOVERY_RESULT_OK;
 if (board_recovery_snapshot(&snapshot)!=0) result=BOARD_RECOVERY_RESULT_IO;
 if (generation==0U || (snapshot.flags&BOARD_RECOVERY_SUPPORTED)==0U) {
  snapshot.flags&=(uint8_t)~BOARD_RECOVERY_SUPPORTED;
  result=BOARD_RECOVERY_RESULT_UNSUPPORTED;
 }
 /* These four domains own only cache Startup, not Idle/output recovery.
  * Reject unsupported intent before loading/committing a floor or calling owner. */
 if (request.operation==BOARD_RECOVERY_RECOVER && request.desired_state!=0U)
  result=BOARD_RECOVERY_RESULT_INVALID;
 if (request.operation==BOARD_RECOVERY_QUERY && result==BOARD_RECOVERY_RESULT_OK &&
     !board_recovery_floor_is_healthy()) result=BOARD_RECOVERY_RESULT_IO;
 if (request.operation==BOARD_RECOVERY_RECOVER && result==BOARD_RECOVERY_RESULT_OK) {
  uint64_t now=0U;
  uint8_t duplicate=previous_request_valid &&
      memcmp(previous_request+4U,packet->payload+4U,8U)==0 &&
      memcmp(previous_request+24U,packet->payload+24U,8U)==0;
  if (generation!=request.expected_boot_generation) result=BOARD_RECOVERY_RESULT_STALE;
  else if (duplicate && memcmp(previous_request,packet->payload,BOARD_RECOVERY_REQUEST_SIZE)!=0)
   result=BOARD_RECOVERY_RESULT_INVALID;
  else if (packet->timestamp==0U || seds_router_get_network_time_ms(router,&now)!=SEDS_OK ||
           (now>=packet->timestamp ? now-packet->timestamp>5000U : packet->timestamp-now>1000U))
   result=BOARD_RECOVERY_RESULT_STALE;
  else if (!duplicate && snapshot.abort_epoch!=request.expected_abort_epoch)
   result=BOARD_RECOVERY_RESULT_STALE;
  else if (!duplicate && !board_recovery_floor_allows_recovery_time(request.issuer_session,packet->timestamp))
   result=BOARD_RECOVERY_RESULT_STALE;
  else if (!duplicate && !board_recovery_floor_commit(request.issuer_session,packet->timestamp))
   result=BOARD_RECOVERY_RESULT_IO;
  else {
   if (!duplicate) {
    memcpy(previous_request,packet->payload,BOARD_RECOVERY_REQUEST_SIZE);
    previous_request_valid=1U;
   }
   int32_t applied=board_recovery_apply_safe(request.desired_state,request.expected_abort_epoch,&snapshot);
   result=applied==0 ? BOARD_RECOVERY_RESULT_OK : applied==-14 ? BOARD_RECOVERY_RESULT_PENDING : applied==-9 ? BOARD_RECOVERY_RESULT_IO : BOARD_RECOVERY_RESULT_STALE;
  }
 }
 uint8_t status[BOARD_RECOVERY_STATUS_SIZE];
 board_recovery_encode_status(status,result,snapshot.actual_state,snapshot.flags,&request,
                              generation,snapshot.abort_epoch,uptime_ms,snapshot.actual_outputs_mask);
 /* Status metadata targets only GroundStation; ACK is not state confirmation. */
 return seds_router_log_queue_typed(router,SEDS_DT_BOARD_STATE_STATUS,status,
                                   sizeof status,sizeof(uint8_t),SEDS_EK_UNSIGNED);
}
