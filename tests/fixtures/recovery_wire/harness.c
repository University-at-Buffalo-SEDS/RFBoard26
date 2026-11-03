#include "main.h"
#include "tx_api.h"
#include "persistent_store.h"
#include "board_recovery.h"
#include "board_recovery_boot.h"
#include "board_recovery_transport.h"
#include "board_recovery_floor.h"
#include "board_recovery_wire.h"
#include "flight_state_cache.h"
#include "launchcore/storage.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static uint8_t disk[4096],response[40];
static bool runtime,fixture=true,fail_lock,fail_unlock,fail_read,owner_fail;static uint32_t fail_key;
static TX_MUTEX*physical;
static unsigned media_reads,media_writes,app_replies;
static uint64_t clock_ms=10000;
volatile uint32_t g_telemetry_discovery_seen;
struct RccFake test_rcc;struct PwrFake test_pwr;struct TampFake test_tamp;
void test_dsb(void){}
uint32_t HAL_GetTick(void){return 123;}
void*tx_thread_identify(void){return runtime?(void*)1:NULL;}
UINT tx_mutex_create(TX_MUTEX*m,const char*n,UINT p){(void)n;(void)p;physical=m;m->locked=0;return fail_unlock?TX_MUTEX_ERROR:TX_SUCCESS;}
UINT tx_mutex_get(TX_MUTEX*m,ULONG wait){(void)wait;if(fail_lock)return TX_MUTEX_ERROR;assert(!m->locked);m->locked=1;return TX_SUCCESS;}
UINT tx_mutex_put(TX_MUTEX*m){assert(m->locked);m->locked=0;return fail_unlock?TX_MUTEX_ERROR:TX_SUCCESS;}
static const launchcore_storage_layout_t layout={.persistent_data_base=0,.persistent_data_size=4096,.persistent_data_erase_size=2048,.persistent_data_write_size=8};
static void check_owner(void){assert(fixture||!runtime||(physical&&physical->locked));}
static launchcore_storage_status_t init_media(void){return LAUNCHCORE_STORAGE_OK;}
static const launchcore_storage_layout_t*get_layout(void){return &layout;}
static launchcore_storage_status_t read_media(uint32_t a,void*p,uint32_t n){check_owner();assert(a<=4096&&n<=4096-a);media_reads++;if(fail_read){fail_read=false;return LAUNCHCORE_STORAGE_ERR_READ;}memcpy(p,disk+a,n);return LAUNCHCORE_STORAGE_OK;}
static launchcore_storage_status_t erase_media(uint32_t a,uint32_t n){check_owner();assert(a<=4096&&n<=4096-a);media_writes++;memset(disk+a,255,n);return LAUNCHCORE_STORAGE_OK;}
static launchcore_storage_status_t write_media(uint32_t a,const void*p,uint32_t n){check_owner();assert(a<=4096&&n<=4096-a);media_writes++;uint32_t key=0;if(n>=24)memcpy(&key,(const uint8_t*)p+4,4);if(key&&key==fail_key)return LAUNCHCORE_STORAGE_ERR_WRITE;for(uint32_t i=0;i<n;i++){assert((disk[a+i]&((const uint8_t*)p)[i])==((const uint8_t*)p)[i]);disk[a+i]=((const uint8_t*)p)[i];}return LAUNCHCORE_STORAGE_OK;}
const launchcore_storage_driver_t launchcore_board_storage_driver={.init=init_media,.read=read_media,.erase=erase_media,.write=write_media,.layout=get_layout};
static void sdk_unlocked(void){assert(!physical||!physical->locked);}
bool seds_router_sender_matches_hostname(const SedsRouter*r,const char*s,size_t n,const char*name,size_t len){sdk_unlocked();assert(r&&len==2&&!memcmp(name,"GS",2));return s&&((n==2&&!memcmp(s,"GS",2))||(n==10&&!memcmp(s,"@addr:1234",10)));}
SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){sdk_unlocked();assert(r);*out=clock_ms;return SEDS_OK;}
SedsResult seds_router_log_queue_typed(SedsRouter*r,SedsDataType type,const void*p,size_t count,size_t width,SedsElemKind kind){sdk_unlocked();assert(r&&type==201&&count==40&&width==1&&kind==SEDS_EK_UNSIGNED);memcpy(response,p,40);assert((response[3]&32U)==EXPECTED_CAP);app_replies++;return SEDS_OK;}
SedsResult seds_router_enable_network_variable(SedsRouter*r,SedsDataType type,bool read,bool write){sdk_unlocked();(void)r;assert(type==SEDS_DT_FLIGHT_STATE&&read&&!write);return SEDS_OK;}
SedsResult seds_router_on_network_variable_update(SedsRouter*r,SedsDataType type,SedsResult(*cb)(const SedsPacketView*,void*),void*user){sdk_unlocked();(void)r;(void)cb;assert(type==SEDS_DT_FLIGHT_STATE&&!user);return SEDS_OK;}
SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*r,SedsDataType type,bool*out){sdk_unlocked();(void)r;assert(type==SEDS_DT_FLIGHT_STATE&&!out);return owner_fail?SEDS_IO:SEDS_OK;}
SedsResult seds_router_request_managed_variable(SedsRouter*r,SedsDataType type){sdk_unlocked();(void)r;assert(type==SEDS_DT_FLIGHT_STATE);return SEDS_OK;}
#ifdef FC_DOMAIN
/* Bounded physical-owner boundary: valid epoch, receive loop ready, outputs
 * already LOW/retained. The mutation under test precedes owner application. */
static uint32_t epoch=1;static uint8_t actual_state=15;
bool fc_recovery_prepare_startup(uint32_t expected){assert(expected==epoch);return true;}
bool fc_recovery_reset_failed(void){return false;}
int32_t board_recovery_snapshot(BoardRecoverySnapshot*out){*out=(BoardRecoverySnapshot){.actual_state=actual_state,.flags=31,.abort_epoch=epoch};return 0;}
int32_t board_recovery_apply_safe(uint8_t desired,uint32_t expected,BoardRecoverySnapshot*out){assert(!desired);if(owner_fail)return -9;if(expected+1U==epoch&&actual_state==0)return board_recovery_snapshot(out);assert(expected==epoch);actual_state=0;epoch++;return board_recovery_snapshot(out);}
#endif
static void put64(uint8_t*p,uint64_t value){for(unsigned i=0;i<8;i++)p[i]=(uint8_t)(value>>(8*i));}

static uint64_t get64(const uint8_t*p){uint64_t v=0;for(unsigned i=0;i<8;i++)v|=(uint64_t)p[i]<<(8*i);return v;}
static void send(SedsRouter*r,SedsPacketView*p){unsigned n=app_replies;assert(board_recovery_handle(r,p,SEDS_EP_BOARD_RECOVERY,123)==0&&app_replies==n+1);}
static uint32_t response_epoch(void){uint32_t n;memcpy(&n,response+20,4);return n;}

#include "board_commissioning.h"
#include "commission_record.h"
ULONG tx_time_get(void){return 1;}
static void request(SedsRouter*r,SedsPacketView*p,bool service){unsigned before=app_replies;assert(board_recovery_handle(r,p,SEDS_EP_BOARD_RECOVERY,123)==0);if(service)board_commissioning_service();assert(app_replies==before+1);}
int main(int argc,char**argv){assert(argc==2);(void)send;(void)response_epoch;
 bool virgin=!strcmp(argv[1],"virgin");bool healthy=strcmp(argv[1],"virgin")&&strcmp(argv[1],"floor");memset(disk,255,sizeof disk);launchcore_storage_set_driver(&launchcore_board_storage_driver);launchcore_persist_strict_info_t info;uint64_t prior=41;uint8_t original=1;
 if(!virgin){assert(launchcore_persist_init()==0);assert(launchcore_persist_set(0x42524745U,&prior,8)==0);assert(launchcore_persist_set(0x46535445U,&original,1)==0);}
 if(healthy){uint64_t floor[2]={77,100};assert(launchcore_persist_set(0x4252464cU,floor,16)==0);}
 fixture=false;(void)persistent_store_boot_begin();uint64_t generation=board_recovery_boot_generation();if(!strcmp(argv[1],"query_fault"))fail_read=true;(void)board_recovery_floor_prime_boot();board_commissioning_probe_boot();flight_state_cache_restore();assert(persistent_store_runtime_start()==0);runtime=true;if(!healthy||!strcmp(argv[1],"query_fault")){assert(!persistent_store_controls_ready());}else{assert(persistent_store_controls_ready());}
 int cookie=0;SedsRouter*r=(SedsRouter*)&cookie;assert(flight_state_cache_init(r)==0);board_recovery_transport_set_router(r);uint32_t endpoint=SEDS_EP_BOARD_RECOVERY;uint8_t body[32]={1,0};put64(body+4,1);put64(body+24,77);SedsPacketView packet={.ty=200,.sender="GS",.sender_len=2,.endpoints=&endpoint,.num_endpoints=1,.payload=body,.payload_len=32,.timestamp=10000};
 if(healthy){

  if(!strcmp(argv[1],"query_fault")){unsigned reads=media_reads,writes=media_writes;request(r,&packet,false);assert(response[1]==7&&reads==media_reads&&writes==media_writes&&!persistent_store_controls_ready());puts("strict boot-prime fault, RAM-only QUERY IO, no mutation PASS");return 0;}
  unsigned rd=media_reads,wr=media_writes;request(r,&packet,false);assert(response[1]==0&&rd==media_reads&&wr==media_writes);
  body[1]=1;put64(body+4,2);put64(body+12,generation);body[20]=1;packet.timestamp=clock_ms=10001;

  unsigned before_reads=media_reads,before_writes=media_writes,before_replies=app_replies;
  if(!strcmp(argv[1],"authority")){
   packet.sender="PB";assert(board_recovery_handle(r,&packet,endpoint,123)==SEDS_PERMISSION_DENIED&&app_replies==before_replies);packet.sender="@addr:1234";packet.sender_len=10;
  }
  if(!strcmp(argv[1],"unsupported_idle")){body[2]=1;request(r,&packet,false);assert(response[1]==6&&media_reads==before_reads&&media_writes==before_writes&&response_epoch()==1&&response[2]==1&&!(response[3]&18U));body[2]=0;}
  if(!strcmp(argv[1],"stale")){body[20]=2;request(r,&packet,false);assert(response[1]==4&&media_reads==before_reads&&media_writes==before_writes&&response_epoch()==1&&response[2]==1);return 0;}
  if(!strcmp(argv[1],"io"))fail_key=0x4252464cU;
  if(!strcmp(argv[1],"pending")){unsigned replies=app_replies;assert(board_recovery_handle(r,&packet,endpoint,123)==SEDS_OK&&app_replies==replies);assert(media_reads==before_reads&&media_writes==before_writes&&!persistent_store_controls_ready());BoardRecoverySnapshot held={0};assert(board_recovery_snapshot(&held)==0&&held.abort_epoch==1&&held.actual_state==1&&(held.flags&1U)&&!(held.flags&18U));assert(board_recovery_handle(r,&packet,endpoint,123)==SEDS_OK&&app_replies==replies);}
  request(r,&packet,true);
  if(!strcmp(argv[1],"io")){assert(response[1]==7&&response[3]&1U&&!(response[3]&18U)&&!persistent_store_controls_ready()&&response_epoch()==2&&response[2]==1);unsigned reads=media_reads,writes=media_writes;fail_key=0;request(r,&packet,false);assert(response[1]==7&&reads==media_reads&&writes==media_writes);puts("actual NVM IO stays held, exact duplicate performs no IO PASS");return 0;}

  assert(response[1]==0&&response[2]==0&&response_epoch()==2&&!(response[3]&19U)&&persistent_store_controls_ready()&&!persistent_store_commission_pending());
  uint8_t marker[40];size_t size=40;assert(persistent_store_enter());assert(launchcore_persist_get_strict(COMMISSION_RECORD_KEY,marker,&size,&info)==0);assert(persistent_store_leave());CommissionRecord record;assert(commission_record_decode(marker,size,&record)&&record.kind==1&&record.state==0&&record.generation==generation&&record.nonce==2&&record.source==10001);
  uint8_t original_request[32];memcpy(original_request,body,32);body[1]=0;put64(body+4,3);put64(body+12,0);memset(body+20,0,4);packet.timestamp=clock_ms=10002;unsigned query_reads=media_reads,query_writes=media_writes;request(r,&packet,false);assert(response[1]==0&&response[2]==0&&response_epoch()==2&&!(response[3]&1U)&&get64(response+12)==generation&&media_reads==query_reads&&media_writes==query_writes);memcpy(body,original_request,32);packet.timestamp=10001;
  wr=media_writes;request(r,&packet,false);assert(response[1]==0&&media_writes==wr);packet.timestamp=10002;request(r,&packet,false);assert(response[1]==6&&media_writes==wr);
  puts("normal healthy one-request durablePENDING->ownedapply->COMPLETE PASS");return 0;
 }
 unsigned reads=media_reads,writes=media_writes;request(r,&packet,false);assert(reads==media_reads&&writes==media_writes&&response[1]==7&&!(response[3]&1U)&&!(response[3]&18U));uint64_t expected=virgin?UINT64_MAX:generation;assert(get64(response+12)==expected);
 body[1]=1;put64(body+4,2);put64(body+12,expected);body[20]=1;packet.timestamp=clock_ms=10001;request(r,&packet,true);assert(response[1]==7&&response[3]&1U&&!(response[3]&18U)&&!persistent_store_controls_ready()&&persistent_store_commission_pending());generation=get64(response+12);assert(generation==(virgin?1:42));assert(flight_state_cache_has_value()==!virgin);if(!virgin)assert(flight_state_cache_value()==1);
 uint8_t marker[40];size_t size=40;assert(persistent_store_enter());assert(launchcore_persist_get_strict(COMMISSION_RECORD_KEY,marker,&size,&info)==0);assert(persistent_store_leave());CommissionRecord history;assert(commission_record_decode(marker,size,&history)&&history.state==1&&history.generation==expected&&history.source==10001&&history.nonce==2&&history.issuer==77);
 writes=media_writes;body[1]=0;put64(body+4,3);put64(body+12,0);body[20]=0;packet.timestamp=10000;request(r,&packet,false);assert(response[1]==7&&media_writes==writes);body[1]=1;put64(body+4,4);put64(body+12,generation);body[20]=1;packet.timestamp=clock_ms=10002;request(r,&packet,false);assert(response[1]==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes&&!persistent_store_controls_ready());
 body[1]=0;put64(body+4,5);put64(body+12,0);body[20]=0;packet.timestamp=clock_ms=10003;reads=media_reads;request(r,&packet,false);assert(response[1]==0&&response[3]&1U&&reads==media_reads&&writes==media_writes);
 body[1]=1;put64(body+4,6);put64(body+12,generation);body[20]=1;packet.timestamp=clock_ms=10004;request(r,&packet,true);assert(response[1]==0&&!(response[3]&19U)&&response[2]==0&&response_epoch()==2&&persistent_store_controls_ready()&&!persistent_store_commission_pending());
 size=40;assert(persistent_store_enter());assert(launchcore_persist_get_strict(COMMISSION_RECORD_KEY,marker,&size,&info)==0);assert(persistent_store_leave());assert(commission_record_decode(marker,size,&history)&&history.state==0&&history.generation==expected&&history.source==10001);
 writes=media_writes;request(r,&packet,false);assert(response[1]==0&&media_writes==writes);packet.timestamp=10005;request(r,&packet,false);assert(response[1]==6&&media_writes==writes);
 puts("actual wire->metadataowner->strictjournal firstlegIO retainedhold; freshsecondleg COMPLETE PASS");return 0;
}
