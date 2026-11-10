
#include "sedsnet.h"
#include "ota_stream.h"
#include "flight_state_cache.h"
#include "board_recovery.h"
#include "board_commissioning.h"
#include "board_recovery_transport.h"
#include "board_recovery_floor.h"
#include "persistent_store.h"
#include "launchcore/storage.h"
#include "launchcore/update.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static SedsP2pStreamHandlerFn ota_callback;static SedsResult(*cache_callback)(const SedsPacketView*,void*);
static uint64_t clock_ms=1000;static uint32_t irq_mask;static unsigned physical,reset_calls,begin_calls,write_calls,finish_calls,abort_calls,confirm_calls;
static bool healthy=true,floor_healthy=true,leave_fails,fail_begin;static uint64_t generation=42;static int lc_hook;static bool cancel_during_apply;
static void observed_launch(void);
static uint8_t stored_phase=1;static bool pending;static uint32_t update_size,update_offset;static uint8_t reply[13];
volatile uint32_t g_telemetry_discovery_seen;
const launchcore_storage_driver_t launchcore_board_storage_driver={1};
uint64_t telemetry_now_ms(void){return clock_ms;}uint32_t HAL_GetTick(void){return(uint32_t)clock_ms;}
uint32_t __get_PRIMASK(void){return irq_mask;}void __disable_irq(void){irq_mask=1;}void __set_PRIMASK(uint32_t v){irq_mask=v;}
void NVIC_SystemReset(void){++reset_calls;}
bool persistent_store_enter(void){assert(!physical);physical=1;return true;}bool persistent_store_leave(void){assert(physical);physical=0;return !leave_fails;}
#ifdef DATA4_DAQ
persistent_store_owner_t persistent_store_try_enter(void){if(physical)return PERSISTENT_STORE_OWNER_BUSY;physical=1;return PERSISTENT_STORE_OWNER_ACQUIRED;}
#endif
int persistent_store_init(void){return 0;}int persistent_store_get(uint32_t key,void*out,size_t*len){assert(key==0x46535445u&&*len==1);*(uint8_t*)out=stored_phase;return 0;}
int persistent_store_set(uint32_t key,const void*value,size_t len){assert(key==0x46535445u&&len==1);stored_phase=*(const uint8_t*)value;return 0;}
bool persistent_store_controls_ready(void){return healthy&&!pending;}bool persistent_store_recovery_ready(void){return healthy;}bool persistent_store_commission_pending(void){return pending;}
void persistent_store_hold_commissioning(void){pending=true;}bool persistent_store_publish_commission_complete(void){pending=false;return true;}void persistent_store_note_failure(void){}
uint64_t board_commissioning_query_generation(void){return generation;}uint8_t board_recovery_floor_cached_health(void){return floor_healthy;}
uint8_t board_recovery_floor_active(void){return 1;}uint8_t board_recovery_floor_is_healthy(void){return 1;}uint8_t board_recovery_command_time_allowed(uint64_t stamp){return stamp>100&&stamp!=UINT64_MAX;}
bool seds_router_sender_matches_hostname(const SedsRouter*r,const char*s,size_t n,const char*name,size_t count){assert(r&&!irq_mask);return s&&n==count&&!memcmp(s,name,n);}
SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){assert(r&&!irq_mask);*out=clock_ms;return 0;}
SedsResult seds_router_enable_network_variable(SedsRouter*r,uint32_t ty,bool a,bool b){assert(r&&ty==SEDS_DT_FLIGHT_STATE&&a&&!b);return 0;}
SedsResult seds_router_on_network_variable_update(SedsRouter*r,uint32_t ty,SedsResult(*fn)(const SedsPacketView*,void*),void*u){assert(r&&ty==SEDS_DT_FLIGHT_STATE&&!u);cache_callback=fn;return 0;}
SedsResult seds_router_request_managed_variable(SedsRouter*r,uint32_t ty){(void)r;(void)ty;return 0;}
SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*r,uint32_t ty,bool*out){assert(r&&ty==SEDS_DT_FLIGHT_STATE&&!out);return 0;}
SedsResult seds_router_bind_p2p_stream_port(SedsRouter*r,uint16_t port,SedsP2pStreamHandlerFn cb,void*u){assert(r&&port==4510&&!u);ota_callback=cb;return 0;}
SedsResult seds_router_send_p2p_stream(SedsRouter*r,uint32_t id,const uint8_t*p,size_t n){assert(r&&id&&n==13&&!physical&&!irq_mask);memcpy(reply,p,n);return 0;}
void launchcore_storage_set_driver(const launchcore_storage_driver_t*d){assert(physical&&d==&launchcore_board_storage_driver);}int launchcore_storage_init(void){assert(physical);return 0;}
int launchcore_delta_update_begin(uint32_t n){assert(physical&&!irq_mask);++begin_calls;if(lc_hook==1)observed_launch();if(fail_begin)return LAUNCHCORE_ERR_FLASH;update_size=n;update_offset=0;return 0;}
int launchcore_delta_update_write(const void*p,size_t n){assert(physical&&!irq_mask&&p&&n<=update_size-update_offset);++write_calls;if(lc_hook==2)observed_launch();if(lc_hook==5){lc_hook=0;extern void reset_from_hook(void);reset_from_hook();}update_offset+=(uint32_t)n;return 0;}
int launchcore_delta_update_finish(void){assert(physical&&!irq_mask&&update_offset==update_size);++finish_calls;if(lc_hook==3)observed_launch();return 0;}
int launchcore_delta_update_abort(void){assert(physical&&!irq_mask);++abort_calls;return 0;}int launchcore_confirm_boot(void){assert(physical&&!irq_mask);++confirm_calls;if(lc_hook==4)observed_launch();return 0;}
static SedsP2pStreamEventView event={.stream_id=9,.peer_stream_id=11,.peer_hostname="GS",.peer_hostname_len=2,.peer_address=321,.local_port=4510,.peer_port=1234};
static SedsResult data(const uint8_t*p,size_t n){event.kind=SEDS_P2P_STREAM_DATA;event.payload=p;event.payload_len=n;return ota_callback(&event,0);}
static void phase(uint8_t p){SedsPacketView packet={.ty=SEDS_DT_FLIGHT_STATE,.sender="GS",.sender_len=2,.payload=&p,.payload_len=1,.timestamp=clock_ms};assert(cache_callback(&packet,0)==0);BoardRecoverySnapshot out;assert(board_recovery_snapshot(&out)==0&&out.actual_state==p);}

static SedsResult phase_raw(uint8_t p){SedsPacketView packet={.ty=SEDS_DT_FLIGHT_STATE,.sender="GS",.sender_len=2,.payload=&p,.payload_len=1,.timestamp=clock_ms};return cache_callback(&packet,0);}
/* A launch observed while OTA owns flash is applied and acknowledged (its
 * inhibition is immediate); only its persistence waits for the owner. */
static void observed_launch(void){lc_hook=0;assert(physical&&!irq_mask);assert(phase_raw(7)==0&&flight_state_cache_value()==7);}
void reset_from_hook(void){event.kind=SEDS_P2P_STREAM_RESET;assert(ota_callback(&event,0)==0);}
SedsResult flight_state_cache_apply_network_update(const SedsPacketView*p){(void)p;if(cancel_during_apply){cancel_during_apply=false;reset_from_hook();}return 0;}
static void accept(void){event.kind=SEDS_P2P_STREAM_ACCEPTED;assert(ota_callback(&event,0)==0);}
static void begin(void){uint8_t b[]={1,128,0,0,0};assert(data(b,sizeof b)==0);}
static void chunks(void){uint8_t c[117]={2};assert(data(c,sizeof c)==0);uint8_t t[21]={2,112,0,0,0};assert(data(t,sizeof t)==0);}
static void finish(void){uint8_t op=3;assert(data(&op,1)==0);}
static void complete(void){begin();chunks();finish();assert(begin_calls==1&&write_calls==2&&finish_calls==1);}
static void poll_reset(unsigned n){clock_ms+=250;ota_stream_poll();assert(reset_calls==n);}
static void denied(void){event.kind=SEDS_P2P_STREAM_ACCEPTED;assert(ota_callback(&event,0)!=0);}
static void recover(void){BoardRecoverySnapshot x;assert(board_recovery_snapshot(&x)==0);uint32_t epoch=x.abort_epoch;assert(board_recovery_commission_prepare(epoch)==0);assert(board_recovery_commission_apply(epoch)==0);assert(board_recovery_commission_finish(epoch,epoch+1));}
int main(int argc,char**argv){assert(argc==2);const char*which=argv[1];
 if(!strcmp(which,"restored_launch"))stored_phase=7;
 SedsRouter router={1};board_recovery_transport_set_router(&router);assert(flight_state_cache_init(&router)==0);assert(ota_stream_init(&router)==0);
 if(!strcmp(which,"restored_launch")){phase(1);denied();assert(!begin_calls&&!reset_calls);return 0;}
 if(!strcmp(which,"unsafe_phase")){phase(2);denied();assert(!begin_calls);return 0;}
 if(!strcmp(which,"unknown_identity")){generation=0;denied();assert(!begin_calls);return 0;}
 if(!strcmp(which,"exhausted_identity")){generation=UINT64_MAX;denied();assert(!begin_calls);return 0;}
 if(!strcmp(which,"wrong_principal")){event.peer_hostname="OTHER";event.peer_hostname_len=5;denied();assert(!begin_calls);return 0;}
 accept();
 if(!strcmp(which,"malformed")){assert(data(0,1)!=0);uint8_t b[]={1,1};assert(data(b,2)==0&&reply[1]==OTA_STREAM_BAD_MESSAGE);assert(!begin_calls);return 0;}
 if(!strcmp(which,"wrong_tuple")){++event.peer_port;uint8_t b[]={1,128,0,0,0};assert(data(b,5)!=0&&!begin_calls);return 0;}
 if(!strcmp(which,"wrong_address")){++event.peer_address;uint8_t b[]={1,128,0,0,0};assert(data(b,5)!=0&&!begin_calls);return 0;}
 if(!strcmp(which,"wrong_peer_stream")){++event.peer_stream_id;uint8_t b[]={1,128,0,0,0};assert(data(b,5)!=0&&!begin_calls);return 0;}
 if(!strcmp(which,"reentrant_begin")){lc_hook=1;begin();assert(reply[1]==OTA_STREAM_BAD_STATE&&begin_calls==1);uint8_t c[117]={2};assert(data(c,sizeof c)!=0);assert(!write_calls);poll_reset(0);return 0;}
 if(!strcmp(which,"reentrant_write")){begin();lc_hook=2;uint8_t c[117]={2};assert(data(c,sizeof c)==0&&reply[1]==OTA_STREAM_BAD_STATE);uint8_t op=3;assert(data(&op,1)!=0);poll_reset(0);assert(write_calls==1&&!finish_calls&&!abort_calls&&!confirm_calls);phase(7);phase(1);denied();return 0;}
 if(!strcmp(which,"active_abort")){begin();uint8_t op=4;assert(data(&op,1)==0&&abort_calls==1);poll_reset(0);return 0;}
 if(!strcmp(which,"bad_offset")){begin();uint8_t c[13]={2,8};assert(data(c,sizeof c)==0&&reply[1]==OTA_STREAM_BAD_OFFSET&&!write_calls);return 0;}
 if(!strcmp(which,"reentrant_finish")){begin();chunks();lc_hook=3;finish();assert(reply[1]==OTA_STREAM_BAD_STATE&&finish_calls==1);poll_reset(0);assert(!abort_calls);return 0;}
 if(!strcmp(which,"busy_reset")){begin();lc_hook=5;uint8_t c[117]={2};assert(data(c,sizeof c)==0&&reply[1]==OTA_STREAM_BAD_STATE);poll_reset(0);assert(write_calls==1&&!abort_calls&&!finish_calls);return 0;}
 complete();
 if(!strcmp(which,"same_session_begin")){begin();assert(begin_calls==2);poll_reset(0);chunks();finish();assert(write_calls==4&&finish_calls==2);poll_reset(1);}
 else if(!strcmp(which,"same_session_begin_failure")){fail_begin=true;begin();assert(begin_calls==2&&reply[1]==OTA_STREAM_STORAGE_ERROR);poll_reset(0);assert(write_calls==2&&finish_calls==1);}
 else if(!strcmp(which,"closed_reset")){event.kind=SEDS_P2P_STREAM_CLOSED;assert(ota_callback(&event,0)==0);event.kind=SEDS_P2P_STREAM_RESET;assert(ota_callback(&event,0)==0);poll_reset(0);}
 else if(!strcmp(which,"unhealthy_reset")){healthy=false;event.kind=SEDS_P2P_STREAM_RESET;assert(ota_callback(&event,0)==0);healthy=true;poll_reset(0);assert(!abort_calls);}
 else if(!strcmp(which,"unhealthy_abort")){healthy=false;uint8_t op=4;assert(data(&op,1)==0);healthy=true;poll_reset(0);assert(!abort_calls);}
 else if(!strcmp(which,"unhealthy_close")){healthy=false;event.kind=SEDS_P2P_STREAM_CLOSED;assert(ota_callback(&event,0)==0);healthy=true;poll_reset(0);assert(!abort_calls);}
 else if(!strcmp(which,"busy_update_reset")){cancel_during_apply=true;phase(1);poll_reset(0);}
 else if(!strcmp(which,"wrong_cancel_tuple")){++event.peer_port;event.kind=SEDS_P2P_STREAM_RESET;assert(ota_callback(&event,0)!=0);--event.peer_port;poll_reset(1);}
 else if(!strcmp(which,"healthy_idle")){poll_reset(1);}
 else if(!strcmp(which,"manual_reset")){BoardRecoverySnapshot before,out;assert(board_recovery_snapshot(&before)==0);assert(board_recovery_apply_safe(0,before.abort_epoch,&out)==0&&out.actual_state==0&&out.abort_epoch==before.abort_epoch+1);poll_reset(0);}
 else if(!strcmp(which,"finish_launch")){++clock_ms;phase(7);poll_reset(0);}
 else if(!strcmp(which,"launch_idle")){phase(7);phase(1);poll_reset(0);denied();}
 else if(!strcmp(which,"stop")){phase(15);poll_reset(0);phase(1);denied();}
 else if(!strcmp(which,"corrupt_health")){healthy=false;poll_reset(0);}
 else if(!strcmp(which,"corrupt_floor")){floor_healthy=false;poll_reset(0);}
 else if(!strcmp(which,"epoch_changed")){BoardRecoverySnapshot a;board_recovery_snapshot(&a);assert(board_recovery_apply_safe(0,a.abort_epoch,&a)==0);poll_reset(0);uint8_t op=3;assert(data(&op,1)!=0);}
 else if(!strcmp(which,"identity_changed")){++generation;poll_reset(0);}
 else if(!strcmp(which,"graceful_close")){event.kind=SEDS_P2P_STREAM_CLOSED;assert(ota_callback(&event,0)==0);poll_reset(1);}
 else if(!strcmp(which,"stream_reset")){event.kind=SEDS_P2P_STREAM_RESET;assert(ota_callback(&event,0)==0);poll_reset(0);}
 else if(!strcmp(which,"inactive_abort")){uint8_t op=4;assert(data(&op,1)==0);poll_reset(0);assert(!abort_calls);}
 else if(!strcmp(which,"new_accept")){++event.stream_id;accept();poll_reset(0);}
 else if(!strcmp(which,"clock_rollback")){clock_ms=500;ota_stream_poll();assert(!reset_calls&&!confirm_calls);}
 else if(!strcmp(which,"reentrant_confirm")){clock_ms+=5000;lc_hook=4;ota_stream_poll();assert(confirm_calls==1&&!reset_calls);ota_stream_poll();assert(confirm_calls==1&&!reset_calls);}
 else if(!strcmp(which,"failed_leave")){leave_fails=true;uint8_t op=5;assert(data(&op,1)!=0);leave_fails=false;poll_reset(0);denied();}
 else if(!strcmp(which,"verified_recovery")){phase(7);phase(1);denied();recover();accept();begin();assert(begin_calls==2);}
 else if(!strcmp(which,"no_clear_by_apply")){phase(7);BoardRecoverySnapshot x;board_recovery_snapshot(&x);assert(board_recovery_apply_safe(0,x.abort_epoch,&x)==0);denied();}
 else assert(0);
 assert(!irq_mask&&!physical);return 0;
}
