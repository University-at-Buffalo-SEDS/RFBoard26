#include "persistent_store.h"
#include "ota_stream.h"

#include "board_config.h"
#include "launchcore/storage.h"
#include "launchcore/update.h"
#include "main.h"
#include "telemetry.h"
#include <stdbool.h>
#include <stddef.h>

#define OTA_OP_BEGIN_DELTA 0x01U
#define OTA_OP_CHUNK 0x02U
#define OTA_OP_FINISH 0x03U
#define OTA_OP_ABORT 0x04U
#define OTA_OP_STATUS 0x05U
#define OTA_RESPONSE_FLAG 0x80U
#define OTA_CONFIRM_DELAY_MS 5000ULL
#define OTA_RESET_DELAY_MS 250ULL

extern const launchcore_storage_driver_t launchcore_board_storage_driver;

#include "board_recovery.h"
#include "board_recovery_floor.h"
#include "board_commissioning.h"
#include <string.h>
/* Single existing router owner; short IRQ exclusion also fences managed-state
 * observations. No flash, storage wait, SDK call or clock query while IRQs are
 * masked. A Launch observation may revoke an in-flight call, never cancel it. */
typedef struct { uint32_t revision, epoch; uint64_t generation; } ota_ticket_t;
typedef struct {
  SedsRouter *router;
  uint32_t stream_id, peer_stream_id, peer_address;
  uint16_t peer_port;
  char peer_name[64]; size_t peer_name_len;
  uint32_t expected_offset, declared_size;
  uint64_t initialized_ms, reset_requested_ms;
  ota_ticket_t session;
  bool active, connected, boot_confirmed, reset_requested, cleanup_needed;
} ota_stream_state_t;
static ota_stream_state_t state;
static uint32_t revision = 1U;
static bool inhibited, io_busy, update_busy;
static uint32_t lock_owner(void) { uint32_t p=__get_PRIMASK();__disable_irq();return p; }
static void unlock_owner(uint32_t p) { __set_PRIMASK(p); }
static void revoke_locked(void)
{
  state.reset_requested=false;
  state.connected=false;
  if(state.active)state.cleanup_needed=true;
  if(revision!=UINT32_MAX)++revision;
  else inhibited=true; /* Never recycle a lifetime after exhaustion. */
}
/* RAM-only, fail closed on a busy/unknown floor cache or boot identity. */
static bool healthy_locked(ota_ticket_t *ticket)
{
  BoardRecoverySnapshot snapshot;
  uint64_t generation=board_commissioning_query_generation();
  if(inhibited || update_busy || revision==UINT32_MAX ||
     !persistent_store_controls_ready() || !board_recovery_floor_cached_health() ||
     !generation || generation==UINT64_MAX || board_recovery_snapshot(&snapshot)!=0 ||
     !(snapshot.flags&BOARD_RECOVERY_STATE_VALID) || snapshot.actual_state>1U ||
     !snapshot.abort_epoch || snapshot.abort_epoch==UINT32_MAX)return false;
  if(ticket)*ticket=(ota_ticket_t){revision,snapshot.abort_epoch,generation};
  return true;
}
static bool same_ticket(ota_ticket_t a,ota_ticket_t b)
{ return a.revision==b.revision&&a.epoch==b.epoch&&a.generation==b.generation; }
static bool session_valid_locked(ota_ticket_t ticket)
{
  ota_ticket_t current;
  return healthy_locked(&current)&&same_ticket(current,ticket)&&same_ticket(state.session,ticket);
}
bool ota_stream_state_begin(uint8_t phase)
{
  uint32_t p=lock_owner();
  /* Intent inhibition is deliberately stronger than cache acceptance. */
  if(phase>=7U){inhibited=true;revoke_locked();}
  else if(phase>1U)revoke_locked();
  if(io_busy||update_busy){unlock_owner(p);return false;}
  update_busy=true;unlock_owner(p);return true;
}
void ota_stream_state_end(void)
{uint32_t p=lock_owner();update_busy=false;unlock_owner(p);}
void ota_stream_recovery_complete(uint32_t actual_epoch)
{
  uint32_t p=lock_owner();BoardRecoverySnapshot snapshot;
  if(!io_busy&&!update_busy&&actual_epoch&&actual_epoch!=UINT32_MAX&&
     board_recovery_snapshot(&snapshot)==0&&snapshot.abort_epoch==actual_epoch&&
     snapshot.actual_state==0U&&(snapshot.flags&BOARD_RECOVERY_STATE_VALID)&&
     persistent_store_controls_ready()&&board_recovery_floor_cached_health()){
    revoke_locked();inhibited=revision==UINT32_MAX;
  }
  unlock_owner(p);
}
static bool start_io(ota_ticket_t ticket,bool session)
{
  uint32_t p=lock_owner();ota_ticket_t current;
  bool valid=!io_busy&&healthy_locked(&current)&&same_ticket(ticket,current)&&
      (!session||same_ticket(ticket,state.session));
  if(valid)io_busy=true;
  unlock_owner(p);return valid;
}
static void release_failed(void)
{uint32_t p=lock_owner();inhibited=true;revoke_locked();unlock_owner(p);}
static uint32_t read_u32_le(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static void write_u32_le(uint8_t *p, uint32_t value)
{
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
  p[2] = (uint8_t)(value >> 16);
  p[3] = (uint8_t)(value >> 24);
}

static ota_stream_status_t map_status(launchcore_status_t status)
{
  switch (status) {
  case LAUNCHCORE_OK: return OTA_STREAM_OK;
  case LAUNCHCORE_ERR_NO_SPACE: return OTA_STREAM_NO_SPACE;
  case LAUNCHCORE_ERR_BAD_STATE: return OTA_STREAM_BAD_STATE;
  case LAUNCHCORE_ERR_BAD_IMAGE:
  case LAUNCHCORE_ERR_VERIFY: return OTA_STREAM_BAD_IMAGE;
  case LAUNCHCORE_ERR_FLASH:
  case LAUNCHCORE_ERR_METADATA: return OTA_STREAM_STORAGE_ERROR;
  default: return OTA_STREAM_INTERNAL_ERROR;
  }
}


static SedsResult send_response(uint32_t stream,uint8_t opcode,ota_stream_status_t status)
{
  uint8_t response[13];response[0]=(uint8_t)(opcode|OTA_RESPONSE_FLAG);
  write_u32_le(response+1,(uint32_t)status);
  write_u32_le(response+5,state.expected_offset);
  write_u32_le(response+9,ota_stream_max_patch_size());
  return seds_router_send_p2p_stream(state.router,stream,response,sizeof response);
}
/* Each invocation can linearize at most one LC API call. In particular an old
 * active writer is cleaned under a NEW safe explicit BEGIN, never post-Launch. */
static ota_stream_status_t handle_data(const uint8_t *payload,size_t len,ota_ticket_t ticket)
{
  if(!payload||!len)return OTA_STREAM_BAD_MESSAGE;
  uint8_t opcode=payload[0];uint32_t size=0;size_t chunk=0;
  if(opcode==OTA_OP_BEGIN_DELTA){
    if(len!=5U)return OTA_STREAM_BAD_MESSAGE;
    if(state.active&&!state.cleanup_needed)return OTA_STREAM_BAD_STATE;
    size=read_u32_le(payload+1);
    if(!size||size>ota_stream_max_patch_size())return OTA_STREAM_NO_SPACE;
  }else if(opcode==OTA_OP_CHUNK){
    if(!state.active||len<6U||len>5U+OTA_STREAM_MAX_CHUNK)return OTA_STREAM_BAD_STATE;
    uint32_t offset=read_u32_le(payload+1);chunk=len-5U;
    if(offset!=state.expected_offset)return OTA_STREAM_BAD_OFFSET;
    if(offset>state.declared_size||chunk>state.declared_size-offset)return OTA_STREAM_NO_SPACE;
    if(offset%BOARD_FLASH_WRITE_ALIGNMENT||
       (offset+chunk<state.declared_size&&chunk%BOARD_FLASH_WRITE_ALIGNMENT))return OTA_STREAM_BAD_MESSAGE;
  }else if(opcode==OTA_OP_FINISH){
    if(len!=1U||!state.active)return OTA_STREAM_BAD_STATE;
  }else if(opcode==OTA_OP_ABORT){
    if(len!=1U)return OTA_STREAM_BAD_MESSAGE;
    uint32_t p=lock_owner();state.reset_requested=false;unlock_owner(p);
    if(!state.active)return OTA_STREAM_OK;
  }else if(opcode==OTA_OP_STATUS&&len==1U)return OTA_STREAM_OK;
  else return OTA_STREAM_BAD_MESSAGE;
  if(!start_io(ticket,true))return OTA_STREAM_BAD_STATE;
  if(opcode==OTA_OP_BEGIN_DELTA){
    /* A newly admitted writer retires the prior FINISH timer even if this
     * LC begin subsequently fails. The held IO ticket admits only this call. */
    uint32_t p=lock_owner();state.reset_requested=false;unlock_owner(p);
  }
  bool cleanup=opcode==OTA_OP_BEGIN_DELTA&&state.active;
  launchcore_status_t result;
  if(cleanup||opcode==OTA_OP_ABORT)result=launchcore_delta_update_abort();
  else if(opcode==OTA_OP_BEGIN_DELTA)result=launchcore_delta_update_begin(size);
  else if(opcode==OTA_OP_CHUNK)result=launchcore_delta_update_write(payload+5,chunk);
  else result=launchcore_delta_update_finish();
  uint64_t now=telemetry_now_ms();
  uint32_t p=lock_owner();bool valid=session_valid_locked(ticket);
  /* Bookkeep actual LC state even if observation revoked its authorization. */
  if(result==LAUNCHCORE_OK){
    if(cleanup||opcode==OTA_OP_ABORT||opcode==OTA_OP_FINISH){state.active=false;state.cleanup_needed=false;}
    else if(opcode==OTA_OP_BEGIN_DELTA){state.active=true;state.cleanup_needed=!valid;state.declared_size=size;state.expected_offset=0;}
    else state.expected_offset+=(uint32_t)chunk;
    if(valid&&opcode==OTA_OP_FINISH){state.reset_requested=true;state.reset_requested_ms=now;}
  }
  io_busy=false;unlock_owner(p);
  if(!valid||cleanup)return OTA_STREAM_BAD_STATE;
  return map_status(result);
}
static bool event_authorized(const SedsP2pStreamEventView *event)
{
  return event&&event->stream_id&&event->peer_stream_id&&
      event->local_port==OTA_STREAM_PORT&&event->peer_hostname&&
      event->peer_hostname_len&&event->peer_hostname_len<=sizeof state.peer_name&&
      seds_router_sender_matches_hostname(state.router,event->peer_hostname,
          event->peer_hostname_len,"GS",2U);
}
static bool same_session(const SedsP2pStreamEventView *event)
{
  return event->stream_id==state.stream_id&&event->peer_stream_id==state.peer_stream_id&&
      event->peer_address==state.peer_address&&event->peer_port==state.peer_port&&
      event->peer_hostname_len==state.peer_name_len&&
      !memcmp(event->peer_hostname,state.peer_name,state.peer_name_len);
}
static SedsResult ota_stream_event(const SedsP2pStreamEventView *event,void *user)
{
  (void)user;
  if(!event_authorized(event))return SEDS_BAD_ARG;
  if(event->kind==SEDS_P2P_STREAM_ACCEPTED){
    uint32_t p=lock_owner();
    /* Every new ACCEPT invalidates any old pending reset, even a refused one. */
    revoke_locked();ota_ticket_t ticket;
    if(io_busy||!healthy_locked(&ticket)){unlock_owner(p);return SEDS_ERR;}
    state.stream_id=event->stream_id;state.peer_stream_id=event->peer_stream_id;
    state.peer_address=event->peer_address;state.peer_port=event->peer_port;
    memcpy(state.peer_name,event->peer_hostname,event->peer_hostname_len);
    state.peer_name_len=event->peer_hostname_len;state.session=ticket;state.connected=true;
    unlock_owner(p);return SEDS_OK;
  }
  uint32_t p=lock_owner();
  /* Cancellation identity is independent of positive IO readiness. A health
   * fault/busy owner must not preserve a reset that its peer just canceled. */
  bool identity=same_session(event)&&state.session.revision==revision;
  bool valid=identity&&session_valid_locked(state.session);
  ota_ticket_t ticket=state.session;
  bool connected=state.connected;
  bool close=event->kind==SEDS_P2P_STREAM_CLOSED;
  bool reset=event->kind==SEDS_P2P_STREAM_RESET;
  bool abort=event->kind==SEDS_P2P_STREAM_DATA&&event->payload&&
      event->payload_len==1U&&event->payload[0]==OTA_OP_ABORT;
  if(identity&&(close||reset||abort)){
    bool preserve=close&&valid&&state.reset_requested&&!state.active;
    if(preserve){state.connected=false;unlock_owner(p);return SEDS_OK;}
    state.reset_requested=false;
    if(close||reset)state.connected=false;
    /* Unsafe cancellation is RAM-only; a later NEW safe BEGIN may clean the
     * retired writer. Never call metadata-writing LC abort as unsafe cleanup. */
    if(!valid||io_busy){
      revoke_locked();unlock_owner(p);
      return abort?send_response(event->stream_id,OTA_OP_ABORT,OTA_STREAM_BAD_STATE):SEDS_OK;
    }
  }
  unlock_owner(p);
  if(!identity||(!connected&&!(close||reset))||!valid)return SEDS_ERR;
  if(event->kind==SEDS_P2P_STREAM_DATA){
    if(!event->payload||!event->payload_len)return SEDS_BAD_ARG;
    uint8_t opcode=event->payload[0];
    if(!persistent_store_enter())return SEDS_ERR;
    ota_stream_status_t status=handle_data(event->payload,event->payload_len,ticket);
    if(!persistent_store_leave()){release_failed();return SEDS_ERR;}
    return send_response(event->stream_id,opcode,status);
  }
  if(close||reset){
    bool success=true;
    if(state.active){
      if(!persistent_store_enter())success=false;
      else{
        uint8_t op=OTA_OP_ABORT;
        ota_stream_status_t status=handle_data(&op,1,ticket);
        bool released=persistent_store_leave();
        if(!released){release_failed();success=false;}
        if(status!=OTA_STREAM_OK)success=false;
      }
    }
    p=lock_owner();if(same_ticket(ticket,state.session))revoke_locked();unlock_owner(p);
    if(!success)return SEDS_ERR;
  }
  return SEDS_OK;
}
uint32_t ota_stream_max_patch_size(void){return BOARD_DELTA_SIZE;}
SedsResult ota_stream_init(SedsRouter *router)
{
  if(!router)return SEDS_BAD_ARG;
  if(!persistent_store_enter())return SEDS_ERR;
  launchcore_storage_set_driver(&launchcore_board_storage_driver);
  launchcore_storage_status_t status=launchcore_storage_init();
  bool released=persistent_store_leave();
  if(!released||status!=LAUNCHCORE_STORAGE_OK)return SEDS_ERR;
  /* Do not clear sticky observations on a repeated initializer. */
  state.router=router;state.initialized_ms=telemetry_now_ms();
  uint32_t p=lock_owner();revoke_locked();unlock_owner(p);
  return seds_router_bind_p2p_stream_port(router,OTA_STREAM_PORT,ota_stream_event,NULL);
}
void ota_stream_poll(void)
{
  uint64_t now=telemetry_now_ms();
  if(!persistent_store_enter())return;
  uint32_t p=lock_owner();ota_ticket_t current;
  bool safe=healthy_locked(&current);
  bool confirm=safe&&!state.boot_confirmed&&now>=state.initialized_ms&&
      now-state.initialized_ms>=OTA_CONFIRM_DELAY_MS;
  unlock_owner(p);
  if(confirm&&start_io(current,false)){
    launchcore_status_t result=launchcore_confirm_boot();
    p=lock_owner();ota_ticket_t after;
    if(healthy_locked(&after)&&same_ticket(current,after)&&result==LAUNCHCORE_OK)state.boot_confirmed=true;
    io_busy=false;unlock_owner(p);
  }
  p=lock_owner();
  if(!io_busy&&!state.active&&state.reset_requested&&session_valid_locked(state.session)&&
     now>=state.reset_requested_ms&&now-state.reset_requested_ms>=OTA_RESET_DELAY_MS){
    /* Bounded CMSIS reset-register operation only. No flash or SDK under IRQ. */
    state.reset_requested=false;NVIC_SystemReset();
  }
  unlock_owner(p);
  if(!persistent_store_leave())release_failed();
}
