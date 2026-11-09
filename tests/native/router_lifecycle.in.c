/* Host lifecycle fixture. Production functions are inserted live at test time.
 * Actual SDK archive/codec/cache are linked; hardware callbacks are inert. */
#include "sedsnet.h"
#include <pthread.h>
#include <stdatomic.h>
#include <errno.h>
static void pause_registration(void);
#include "sedsnet_config.h"
#include "flight_state_cache.h"
#include "persistent_store.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* Portable C-call instrumentation. Forwarders are defined before redirect
 * macros so the SDK implementation and wire codec stay the actual library. */
SedsResult __real_seds_router_enable_network_variable(SedsRouter*r,SedsDataType t,bool a,bool b){return seds_router_enable_network_variable(r,t,a,b);}
SedsResult __real_seds_router_on_network_variable_update(SedsRouter*r,SedsDataType t,SedsEndpointHandlerFn f,void*u){return seds_router_on_network_variable_update(r,t,f,u);}
void __real_seds_router_disable_managed_variable(SedsRouter*r,SedsDataType t){seds_router_disable_managed_variable(r,t);}
SedsResult __real_seds_router_invalidate_network_variable_cache(SedsRouter*r,SedsDataType t,bool*out){return seds_router_invalidate_network_variable_cache(r,t,out);}
SedsResult __real_seds_router_request_managed_variable(SedsRouter*r,SedsDataType t){return seds_router_request_managed_variable(r,t);}
SedsRouter*__real_seds_router_new(SedsNowMsFn f,void*u,const SedsLocalEndpointDesc*l,size_t n){return seds_router_new(f,u,l,n);}
SedsRouter*__real_seds_router_new_with_memory(SedsNowMsFn f,void*u,const SedsLocalEndpointDesc*l,size_t n,uint8_t m,uint32_t k,const SedsRuntimeMemoryConfig*c){return seds_router_new_with_memory(f,u,l,n,m,k,c);}
void __real_seds_router_free(SedsRouter*r){seds_router_free(r);}
SedsResult __real_seds_router_poll_timesync(SedsRouter*r,bool*out){return seds_router_poll_timesync(r,out);}
SedsResult __real_seds_router_receive_packed_from_side(SedsRouter*r,uint32_t side,const uint8_t*b,size_t n){return seds_router_receive_packed_from_side(r,side,b,n);}
SedsResult __real_seds_router_receive_packed(SedsRouter*r,const uint8_t*b,size_t n){return seds_router_receive_packed(r,b,n);}

#define seds_router_enable_network_variable __wrap_seds_router_enable_network_variable
#define seds_router_on_network_variable_update __wrap_seds_router_on_network_variable_update
#define seds_router_disable_managed_variable __wrap_seds_router_disable_managed_variable
#define seds_router_invalidate_network_variable_cache __wrap_seds_router_invalidate_network_variable_cache
#define seds_router_request_managed_variable __wrap_seds_router_request_managed_variable
#define seds_router_new __wrap_seds_router_new
#define seds_router_new_with_memory __wrap_seds_router_new_with_memory
#define seds_router_free __wrap_seds_router_free
#define seds_router_poll_timesync __wrap_seds_router_poll_timesync
#define seds_router_receive_packed_from_side __wrap_seds_router_receive_packed_from_side
#define seds_router_receive_packed __wrap_seds_router_receive_packed

volatile uint32_t g_telemetry_discovery_seen;
static uint64_t clock_ms=1000;
static unsigned writes,app_callbacks,registrations,enables,disables,invalidations,refreshes,outputs;
static uint8_t stored=5;
static size_t hosted_schema_allowance,raw_constructor_budget,normalized_constructor_budget;
static uint32_t cold_memory[16];
static unsigned rx_calls;
static SedsResult last_rx_result=SEDS_ERR;
SedsResult __wrap_seds_router_receive_packed_from_side(SedsRouter*r,uint32_t side,const uint8_t*b,size_t n){rx_calls++;return last_rx_result=__real_seds_router_receive_packed_from_side(r,side,b,n);}
SedsResult __wrap_seds_router_receive_packed(SedsRouter*r,const uint8_t*b,size_t n){rx_calls++;return last_rx_result=__real_seds_router_receive_packed(r,b,n);}

static bool fail_write=true,policy=true,fail_enable,fail_callback;
static SedsEndpointHandlerFn original_callback;
static void *original_user;
static SedsRouter *router;
static uint32_t others[2];
static uint8_t other_before[2][2048];
static int32_t other_lengths[2];
static bool check_selective_metadata;
static void topology(char *buffer,size_t capacity){
 int32_t n=seds_router_export_topology_len(router);assert(n>=0&&(size_t)n+1<capacity);
 memset(buffer,0,capacity);assert(seds_router_export_topology(router,buffer,capacity)==SEDS_OK);
}
uint32_t HAL_GetTick(void){return(uint32_t)clock_ms;}
int persistent_store_init(void){return 0;}
int persistent_store_get(uint32_t key,void *p,size_t*n){assert(key==0x46535445U&&*n==1);*(uint8_t*)p=stored;return 0;}
int persistent_store_set(uint32_t key,const void *p,size_t n){assert(key==0x46535445U&&n==1);writes++;if(fail_write)return 2;stored=*(const uint8_t*)p;return 0;}
uint8_t board_recovery_managed_update_allowed(const SedsPacketView *packet){assert(packet);return policy;}
SedsResult __real_seds_router_enable_network_variable(SedsRouter*,SedsDataType,bool,bool);
SedsResult __real_seds_router_on_network_variable_update(SedsRouter*,SedsDataType,SedsEndpointHandlerFn,void*);
void __real_seds_router_disable_managed_variable(SedsRouter*,SedsDataType);
SedsResult __real_seds_router_invalidate_network_variable_cache(SedsRouter*,SedsDataType,bool*);
SedsResult __real_seds_router_request_managed_variable(SedsRouter*,SedsDataType);
static SedsResult observe_callback(const SedsPacketView *p,void *user){(void)user;app_callbacks++;return original_callback(p,original_user);}
SedsResult __wrap_seds_router_enable_network_variable(SedsRouter*r,SedsDataType ty,bool rd,bool wr){if(ty==SEDS_DT_FLIGHT_STATE){enables++;if(fail_enable)return SEDS_HANDLER_ERROR;}return __real_seds_router_enable_network_variable(r,ty,rd,wr);}
SedsResult __wrap_seds_router_on_network_variable_update(SedsRouter*r,SedsDataType ty,SedsEndpointHandlerFn cb,void *user){
 if(ty==SEDS_DT_FLIGHT_STATE){registrations++;pause_registration();if(fail_callback)return SEDS_HANDLER_ERROR;original_callback=cb;original_user=user;return __real_seds_router_on_network_variable_update(r,ty,observe_callback,NULL);}
 return __real_seds_router_on_network_variable_update(r,ty,cb,user);
}
void __wrap_seds_router_disable_managed_variable(SedsRouter*r,SedsDataType ty){disables++;__real_seds_router_disable_managed_variable(r,ty);}
SedsResult __wrap_seds_router_request_managed_variable(SedsRouter*r,SedsDataType ty){refreshes++;return __real_seds_router_request_managed_variable(r,ty);}
SedsResult __wrap_seds_router_invalidate_network_variable_cache(SedsRouter*r,SedsDataType ty,bool*out){
 invalidations++;assert(r==router&&ty==SEDS_DT_FLIGHT_STATE&&out==NULL);
 char before[65536],after[65536];topology(before,sizeof before);
 unsigned old_outputs=outputs,old_refresh=refreshes,old_enables=enables,old_registrations=registrations;
 SedsResult result=__real_seds_router_invalidate_network_variable_cache(r,ty,out);
 topology(after,sizeof after);
 assert(result==SEDS_OK&&!strcmp(before,after));
 assert(outputs==old_outputs&&refreshes==old_refresh&&enables==old_enables&&registrations==old_registrations);
 if(check_selective_metadata)for(unsigned i=0;i<2;i++){
  uint8_t current[2048];int32_t n=seds_router_cached_managed_variable_packed(r,others[i],current,sizeof current);
  assert(n==other_lengths[i]&&n>0&&!memcmp(current,other_before[i],(size_t)n));
 }
 return result;
}
/* Include the actual production cache owner, exposing its static helper only
 * inside this host-only qualification translation unit. No GPIO/task module. */
#include CACHE_SOURCE
#ifndef CACHE_PREPARE_RECOVERY
#define reset_cache_for_proof() flight_state_cache_reset_startup()
#else
#define reset_cache_for_proof() flight_state_cache_prepare_recovery(0)
#endif
static SedsResult inert(const SedsPacketView*p,void*u){(void)p;(void)u;outputs++;return SEDS_OK;}
static int32_t pack(uint32_t ty,uint8_t value,uint8_t *wire,size_t capacity){
 uint32_t endpoint=SEDS_EP_BOARD_RECOVERY;
 SedsPacketView p={.ty=ty,.data_size=1,.sender="GS",.sender_len=2,.endpoints=&endpoint,.num_endpoints=1,.timestamp=900,.payload=&value,.payload_len=1};
 int32_t length=seds_pkt_pack(&p,wire,capacity);assert(length>0);return length;
}


#define UNUSED_FUNCTION __attribute__((unused))
#define HAL_OK 0
#define TX_SUCCESS 0U
#define TX_WAIT_FOREVER (~0U)
#define TX_NULL NULL
#define TX_MUTEX_ID 0x4D555445U
#define BOARD_CAN_MAX_FRAME_BYTES (2U*55U)
#define BOARD_SIDE_TRANSPORT_TEMPLATES 32U
#define FC_CAN_MAX_FRAME_BYTES (2U*55U)
#define FC_SIDE_TRANSPORT_TEMPLATES 4U
#define conditional
#define telemetry_mu g_telemetry_mutex
#define lock_fails g_telemetry_lock_get_fail
#define unlock_fails g_telemetry_lock_put_fail
#define TELEMETRY_ROUTER_RETRY_MS 500U
#define TELEMETRY_CAN_BUS 1
#define SEDS_ENABLE_COMPACT_PACKET_STORE 1
#define CAN_TX_ALIGNED_FRAME_BYTES (2U*55U)
#define RF_SEDSNET_QUEUE_BUDGET 9984U
#define RF_SEDSNET_MAX_RECENT_RX_IDS 16U
#define RF_SEDSNET_STARTING_ALLOCATION 1024U
#define RF_CAN_MAX_FRAME_BYTES (2U*55U)
#define RF_RADIO_MAX_FRAME_BYTES 192U
#define RF_CAN_SIDE_TRANSPORT_TEMPLATES 16U
#define RF_RADIO_SIDE_TRANSPORT_TEMPLATES 16U
#define GATEWAY_UART_MAX_FRAME_BYTES 1024U
#define GATEWAY_SIDE_TRANSPORT_TEMPLATES 16U
typedef struct {SedsRouter*r;uint8_t created;uint64_t start_time;} RouterState;
static RouterState g_router;
static uint32_t g_router_retry_after_ms;
static int32_t g_can_side_id=-1,g_telemetry_init_error_code;
static uint8_t g_can_rx_subscribed;
uint8_t g_radio_rx_subscribed;
int32_t g_radio_side_id=-1;
uint32_t g_discovery_schema_retry_delay_ms,g_telemetry_timesync_queued;
int32_t g_gateway_packet_store_init_result;
uint32_t g_gateway_can_rx_handoff_fail,g_gateway_can_admission_fail,g_gateway_can_rejected_word0,g_gateway_can_rejected_word1,g_gateway_can_rejected_word2,g_gateway_can_rejected_word3,g_gateway_can_queue_fail;
int32_t g_gateway_can_rx_handoff_last;
static __attribute__((unused)) void telemetry_signal_deserialize_failure(void){}
uint32_t g_telemetry_rx_packets,g_telemetry_rx_errors;
unsigned g_telemetry_lock_get_status,g_telemetry_lock_put_status;
uint32_t g_telemetry_rx_ok,g_telemetry_rx_stage,g_telemetry_service_stage,g_telemetry_rx_error_length;
int32_t g_telemetry_rx_last_result;
uint8_t g_telemetry_rx_error_prefix[16];
int32_t g_telemetry_rx_last_error;
static uint8_t g_discovery_schema_announced;
static uint64_t g_discovery_schema_retry_ms;
enum {TELEMETRY_INIT_OK=0,TELEMETRY_INIT_CACHE_FAILED=-1007,TELEMETRY_INIT_SUBSCRIBE_RX_FAILED=-1001,
TELEMETRY_INIT_ROUTER_NEW_FAILED=-1002,TELEMETRY_INIT_ADD_CAN_SIDE_FAILED=-1003,
TELEMETRY_INIT_CONFIGURE_TIMESYNC_FAILED=-1004,TELEMETRY_INIT_OTA_FAILED=-1006};
typedef struct {pthread_mutex_t mutex;unsigned tx_mutex_id;} TX_MUTEX;
typedef unsigned UINT;
typedef uintptr_t TX_THREAD;
static TX_MUTEX g_telemetry_mutex;
static unsigned g_telemetry_mutex_ready=1,g_telemetry_lock_get_fail,g_telemetry_lock_put_fail;
static _Thread_local TX_THREAD self;
static unsigned isr,context_missing,lock_failure,unlock_failure;
static pthread_mutex_t gate=PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t changed=PTHREAD_COND_INITIALIZER;
static unsigned paused,release_pause,pause_enabled,get_attempts,allocations,frees;
static SedsRouter* allocations_list[8];
static unsigned fail_stage;
SedsResult __real_seds_router_poll_timesync(SedsRouter*,bool*);
SedsResult __wrap_seds_router_poll_timesync(SedsRouter*r,bool*queued){if(fail_stage==8)return SEDS_ERR;return __real_seds_router_poll_timesync(r,queued);}
unsigned __get_IPSR(void){return isr;}
TX_THREAD*tx_thread_identify(void){return context_missing?NULL:&self;}
unsigned tx_mutex_get(TX_MUTEX*m,unsigned timeout){
 assert(m==&g_telemetry_mutex&&timeout==TX_WAIT_FOREVER&&!isr);
 pthread_mutex_lock(&gate);get_attempts++;pthread_cond_broadcast(&changed);pthread_mutex_unlock(&gate);
 if(lock_failure)return 99;
 return (unsigned)pthread_mutex_lock(&m->mutex);
}
unsigned tx_mutex_put(TX_MUTEX*m){assert(m==&g_telemetry_mutex);if(unlock_failure)return 99;return (unsigned)pthread_mutex_unlock(&m->mutex);}
static void pause_registration(void){
 pthread_mutex_lock(&gate);
 if(pause_enabled&&!paused){paused=1;pthread_cond_broadcast(&changed);while(!release_pause)pthread_cond_wait(&changed,&gate);}
 pthread_mutex_unlock(&gate);
}
SedsRouter*__real_seds_router_new(SedsNowMsFn,void*,const SedsLocalEndpointDesc*,size_t);
SedsRouter*__wrap_seds_router_new(SedsNowMsFn cb,void*u,const SedsLocalEndpointDesc*l,size_t n){
 SedsRouter*r=__real_seds_router_new(cb,u,l,n);assert(r);pthread_mutex_lock(&gate);
 assert(allocations<8);allocations_list[allocations++]=r;router=r;pthread_cond_broadcast(&changed);pthread_mutex_unlock(&gate);return r;
}
SedsRouter*__real_seds_router_new_with_memory(SedsNowMsFn,void*,const SedsLocalEndpointDesc*,size_t,uint8_t,uint32_t,const SedsRuntimeMemoryConfig*);
SedsRouter*__wrap_seds_router_new_with_memory(SedsNowMsFn cb,void*u,const SedsLocalEndpointDesc*l,size_t n,uint8_t mode,uint32_t key,const SedsRuntimeMemoryConfig*m){
 /* Production supplied the untouched MCU profile. Only this hosted fixture
  * normalizes its std registry RAM; firmware compiled flash tables cost zero
  * dynamic schema bytes. The allowance is measured in a separate process by
  * this same executable/schema/archive, never guessed or added to production. */
 assert(m&&m->max_queue_budget==RF_SEDSNET_QUEUE_BUDGET);
 assert(hosted_schema_allowance<=SIZE_MAX-m->max_queue_budget);
 SedsRuntimeMemoryConfig hosted=*m;
 raw_constructor_budget=m->max_queue_budget;
 hosted.max_queue_budget+=hosted_schema_allowance;
 normalized_constructor_budget=hosted.max_queue_budget;
 assert(hosted.max_recent_rx_ids==m->max_recent_rx_ids&&hosted.starting_queue_size==m->starting_queue_size&&hosted.queue_grow_step==m->queue_grow_step);
 SedsRouter*r=__real_seds_router_new_with_memory(cb,u,l,n,mode,key,&hosted);assert(r);
 assert(seds_router_copy_memory_counters(r,cold_memory,16)==16);
 assert(cold_memory[1]==normalized_constructor_budget);
 if(hosted_schema_allowance)assert(cold_memory[8]==hosted_schema_allowance);
 pthread_mutex_lock(&gate);
 assert(allocations<8);allocations_list[allocations++]=r;router=r;pthread_cond_broadcast(&changed);pthread_mutex_unlock(&gate);return r;
}
void __real_seds_router_free(SedsRouter*);
void __wrap_seds_router_free(SedsRouter*r){frees++;__real_seds_router_free(r);}
static uint64_t tx_raw_now_ms_locked(void){return clock_ms;}
static uint64_t node_now_since_ms(void*u){(void)u;return clock_ms-g_router.start_time;}
static SedsResult tx_send_with_priority(const uint8_t*p,size_t n,uint8_t priority,void*u){(void)p;(void)n;(void)priority;(void)u;return SEDS_OK;}
static void telemetry_can_rx(const uint8_t*p,size_t n,void*u){(void)p;(void)n;(void)u;}
static int can_bus_subscribe_rx(void(*cb)(const uint8_t*,size_t,void*),void*u){assert(cb==telemetry_can_rx&&!u);return HAL_OK;}
static __attribute__((unused)) SedsResult board_packet_store_init(void){g_gateway_packet_store_init_result=fail_stage==1?SEDS_HANDLER_ERROR:SEDS_OK;return g_gateway_packet_store_init_result;}
static __attribute__((unused)) SedsResult daq_calibration_init(SedsRouter*r){assert(r);return fail_stage==6?SEDS_ERR:SEDS_OK;}
static __attribute__((unused)) SedsResult daq_log_clock_init(SedsRouter*r){assert(r);return fail_stage==7?SEDS_ERR:SEDS_OK;}
static __attribute__((unused)) SedsResult sedsnet_hardware_crypto_init(void){return SEDS_OK;}
static __attribute__((unused)) void telemetry_memory_profile_mark(unsigned n){(void)n;}
static __attribute__((unused)) int radio_uart_subscribe_rx(void(*cb)(const uint8_t*,size_t,void*),void*u){assert(cb&&!u);return HAL_OK;}
static __attribute__((unused)) SedsResult radio_tx_send(const uint8_t*p,size_t n,uint8_t pri,void*u){return tx_send_with_priority(p,n,pri,u);}
static __attribute__((unused)) SedsResult telemetry_uart_tx_send(const uint8_t*p,size_t n,void*u){return tx_send_with_priority(p,n,0,u);}
static __attribute__((unused)) void telemetry_uart_set_side_id(int32_t id){(void)id;}
static SedsResult telemetry_configure_timesync_locked(SedsRouter*r){assert(r);return fail_stage==2?SEDS_ERR:SEDS_OK;}
static unsigned underglow_inits;
static __attribute__((unused)) SedsResult av_bay_underglow_init(SedsRouter*r){assert(r);underglow_inits++;return fail_stage==9?SEDS_ERR:SEDS_OK;}
static __attribute__((unused)) SedsResult flight_buzzer_init(SedsRouter*r){assert(r);return SEDS_OK;}
static unsigned ota_inits;
static SedsResult ota_stream_init(SedsRouter*r){assert(r);ota_inits++;return fail_stage==3?SEDS_ERR:SEDS_OK;}
/* Optional-service fault state owned by telemetry.c (RF relay-first init). */
volatile uint32_t g_telemetry_service_faults;
volatile int32_t g_telemetry_service_last_error;
static uint32_t g_telemetry_service_retry_ms;
void board_recovery_transport_set_router(SedsRouter*r){assert(r);}
#define telemetry_radio_rx inert_radio_rx
static __attribute__((unused)) void inert_radio_rx(const uint8_t*p,size_t n,void*u){(void)p;(void)n;(void)u;}
#define Board_Recovery_handler inert
#define control_packet_handler inert
#define Abort_handler inert
#define Flight_State_handler inert
#define Heartbeat_handler inert
SedsResult init_telemetry_router(void);

#define Board_Recovery_handler inert

@@GLOBAL_ENDPOINTS@@

#undef RF_RADIO_MAX_FRAME_BYTES
#define RF_RADIO_MAX_FRAME_BYTES 192U

#undef RF_CAN_SIDE_TRANSPORT_TEMPLATES
#define RF_CAN_SIDE_TRANSPORT_TEMPLATES 16U

#undef RF_RADIO_SIDE_TRANSPORT_TEMPLATES
#define RF_RADIO_SIDE_TRANSPORT_TEMPLATES 16U

#define RX_QUEUED_ONLY 0

#define REQUIRE_RX_COUNTER 0

@@BOARD_DECLARATIONS@@
@@HOOK:telemetry_router_context_ready@@
@@HOOK:telemetry_router_enter@@
@@HOOK:telemetry_router_leave@@
@@HOOK:telemetry_lock@@
@@HOOK:telemetry_unlock@@
@@TELEMETRY:telemetry_start_services_locked@@
@@TELEMETRY:telemetry_retry_services_locked@@
@@TELEMETRY:init_telemetry_router_locked@@
@@TELEMETRY:init_telemetry_router@@
@@TELEMETRY:rx_asynchronous_locked@@
@@TELEMETRY:rx_asynchronous@@

static uint8_t input[2048];static int32_t input_size;
static void*initialize(void*u){(void)u;assert(init_telemetry_router()==SEDS_OK);return NULL;}
static void*receive(void*u){(void)u;rx_asynchronous(input,(size_t)input_size);return NULL;}
int main(int argc,char**argv){
 assert(argc==3||argc==4);assert(seds_schema_register_json_file(argv[1],strlen(argv[1]))==SEDS_OK);
 if(!strcmp(argv[2],"schema_probe")){
  /* Probe-only process, zero application construction/registration/effects. */
  SedsRouter*probe=__real_seds_router_new(NULL,NULL,NULL,0);assert(probe);uint32_t counters[16];
  assert(seds_router_copy_memory_counters(probe,counters,16)==16&&counters[8]>0&&counters[8]<UINT32_MAX);
  assert(allocations==0&&registrations==0&&app_callbacks==0&&writes==0);
  printf("SCHEMA_RAM %u\n",counters[8]);__real_seds_router_free(probe);return 0;
 }
 if(argc==4){char*end=NULL;assert(!strcmp(argv[2],"race"));hosted_schema_allowance=(size_t)strtoull(argv[3],&end,10);assert(end&&!*end&&hosted_schema_allowance>0&&hosted_schema_allowance<UINT32_MAX);}

 pthread_mutexattr_t attr;assert(!pthread_mutexattr_init(&attr));assert(!pthread_mutexattr_settype(&attr,PTHREAD_MUTEX_RECURSIVE));
 assert(!pthread_mutex_init(&g_telemetry_mutex.mutex,&attr));g_telemetry_mutex.tx_mutex_id=TX_MUTEX_ID;
 fail_write=false;
 if(!strcmp(argv[2],"race")||!strcmp(argv[2],"raw_budget")){
  input_size=pack(SEDS_DT_FLIGHT_STATE,0,input,sizeof input);pause_enabled=1;
  pthread_t first,second,rx;assert(!pthread_create(&first,NULL,initialize,NULL));
  pthread_mutex_lock(&gate);while(!paused)pthread_cond_wait(&changed,&gate);assert(g_router.r==NULL&&g_router.created==0&&allocations==1);pthread_mutex_unlock(&gate);
  assert(!pthread_create(&second,NULL,initialize,NULL));assert(!pthread_create(&rx,NULL,receive,NULL));
  pthread_mutex_lock(&gate);while(get_attempts<3)pthread_cond_wait(&changed,&gate);
  assert(allocations==1&&g_router.r==NULL&&app_callbacks==0);release_pause=1;pthread_cond_broadcast(&changed);pthread_mutex_unlock(&gate);
  assert(!pthread_join(first,NULL));assert(!pthread_join(second,NULL));assert(!pthread_join(rx,NULL));
  if(RX_QUEUED_ONLY)assert(app_callbacks==0);
  if(RX_QUEUED_ONLY){assert(telemetry_router_enter());assert(seds_router_process_all_queues(g_router.r)==SEDS_OK);assert(telemetry_router_leave());}
  printf("PROFILE raw=%zu allowance=%zu normalized=%zu cold_shared=%u cold_schema=%u rx_calls=%u rx_result=%d callbacks=%u writes=%u state=%u\n",raw_constructor_budget,hosted_schema_allowance,normalized_constructor_budget,cold_memory[2],cold_memory[8],rx_calls,last_rx_result,app_callbacks,writes,stored);
  printf("FRAME ");for(int32_t i=0;i<input_size;i++)printf("%02x",input[i]);putchar('\n');
  assert(rx_calls==1&&allocations==1&&g_router.r==router&&g_router.created==1&&registrations==1&&enables==1);
  if(!strcmp(argv[2],"raw_budget")){
   assert(hosted_schema_allowance==0&&normalized_constructor_budget==raw_constructor_budget);
   if(cold_memory[2]>raw_constructor_budget){
    /* Real native64 RF registry exceeds the MCU profile: admission must not
     * claim callback/persistence success. Retain this negative permanently. */
    assert(last_rx_result==SEDS_IO&&app_callbacks==0&&writes==0&&stored==5);
    puts("RAW_BUDGET_REFUSED_WITHOUT_CALLBACK_OR_NVM");
   }else{
    /* A smaller supported host ABI can fit the unadjusted profile. */
    assert(last_rx_result==SEDS_OK&&app_callbacks==1&&writes==1&&stored==0);
    puts("RAW_BUDGET_FITS_HOST_ABI");
   }
  }else{
   assert(hosted_schema_allowance==cold_memory[8]);
   assert(normalized_constructor_budget==raw_constructor_budget+hosted_schema_allowance);
   assert(last_rx_result==SEDS_OK);
  assert(allocations==1&&g_router.r==router&&g_router.created==1&&registrations==1&&enables==1&&app_callbacks==1&&writes==1&&stored==0&&(!REQUIRE_RX_COUNTER||g_telemetry_rx_packets+g_telemetry_rx_ok==1));
  }
 }else if(!strcmp(argv[2],"context")){
  isr=1;assert(init_telemetry_router()==SEDS_BAD_ARG&&get_attempts==0&&allocations==0);isr=0;
  context_missing=1;assert(init_telemetry_router()==SEDS_BAD_ARG&&get_attempts==0);context_missing=0;
  g_telemetry_mutex_ready=0;g_telemetry_mutex.tx_mutex_id=0;assert(init_telemetry_router()==SEDS_BAD_ARG&&get_attempts==0);g_telemetry_mutex_ready=1;g_telemetry_mutex.tx_mutex_id=TX_MUTEX_ID;
  lock_failure=1;assert(init_telemetry_router()==SEDS_BAD_ARG&&allocations==0&&g_telemetry_lock_get_fail==1);lock_failure=0;
  assert(telemetry_router_enter());telemetry_lock();assert(telemetry_router_enter());telemetry_lock();telemetry_unlock();assert(telemetry_router_leave());telemetry_unlock();assert(telemetry_router_leave());
  assert(init_telemetry_router()==SEDS_OK);
 }else if(!strcmp(argv[2],"rollover")||!strcmp(argv[2],"zero_deadline")){
  bool zero=!strcmp(argv[2],"zero_deadline");clock_ms=zero?UINT32_MAX-499ULL:UINT32_MAX-250ULL;
  /* Only a router/side construction failure frees and retries now. */
  fail_stage=1;assert(init_telemetry_router()==SEDS_HANDLER_ERROR&&allocations==0&&frees==0);
  uint32_t deadline=g_router_retry_after_ms;assert(deadline==(zero?1U:249U));
  unsigned count=allocations;clock_ms+=499;assert(init_telemetry_router()==SEDS_IO&&allocations==count);
  clock_ms++;fail_stage=0;
  if(zero){assert(init_telemetry_router()==SEDS_IO&&allocations==count);clock_ms++;}
  assert(init_telemetry_router()==SEDS_OK&&allocations==count+1&&g_router_retry_after_ms==0);
 }else if(!strcmp(argv[2],"leave")||!strcmp(argv[2],"primary_error")){
  bool primary=!strcmp(argv[2],"primary_error");fail_stage=primary?1:0;unlock_failure=1;
  assert(init_telemetry_router()==(primary?SEDS_HANDLER_ERROR:SEDS_ERR)&&g_telemetry_lock_put_fail==1);
  assert(primary?(g_router.r==NULL&&frees==0&&allocations==0):(g_router.r==router&&g_router.created==1));
  /* Release the host test kernel resource after injected put failure; this
   * is fixture cleanup, not a claim of automatic firmware lock recovery. */
  unlock_failure=0;assert(!pthread_mutex_unlock(&g_telemetry_mutex.mutex));
 }else{
  unsigned stage=(unsigned)strtoul(argv[2],NULL,10);assert(stage>=1&&stage<=9);
  if(stage!=1){
   /* Relay first: once the router and both sides exist, an optional service
    * failure (timesync config 2, OTA 3, flight-state enable 4 / callback 5,
    * priming timesync poll 8, underglow 9) publishes the router, records a
    * fault and retries in the background. Nothing is freed, so no
    * DISCOVERY_LEAVE storm and CAN<->radio forwarding keeps running. */
   uint32_t bit=stage==2?TELEMETRY_SERVICE_TIMESYNC:stage==3?TELEMETRY_SERVICE_OTA:
     stage==9?TELEMETRY_SERVICE_UNDERGLOW:(stage==4||stage==5)?TELEMETRY_SERVICE_FLIGHT_STATE:0U;
   fail_stage=(stage==2||stage==3||stage==8||stage==9)?stage:0;fail_enable=stage==4;fail_callback=stage==5;
   assert(init_telemetry_router()==SEDS_OK&&g_router.created==1&&g_router.r==router);
   assert(allocations==1&&frees==0&&g_router_retry_after_ms==0);
   assert(g_telemetry_service_faults==bit);
   assert(bit!=TELEMETRY_SERVICE_FLIGHT_STATE||g_cache_router==NULL);
   assert(bit==TELEMETRY_SERVICE_FLIGHT_STATE||g_cache_router==g_router.r);
   /* Later initializers still ran: one failure never skips the others. */
   assert(ota_inits==1&&underglow_inits==1&&(stage==4||stage==5||registrations==1));
   /* The relay is live: frames from a side reach the actual SDK router. */
   input_size=pack(SEDS_DT_FLIGHT_STATE,0,input,sizeof input);
   unsigned before=rx_calls;rx_asynchronous(input,(size_t)input_size);
   assert(rx_calls==before+1&&last_rx_result!=SEDS_BAD_ARG);
   for(unsigned i=0;i<20;i++)assert(init_telemetry_router()==SEDS_OK&&allocations==1);
   /* Bounded background retry: nothing before the deadline, then once. */
   fail_stage=0;fail_enable=fail_callback=false;
   assert(telemetry_router_enter());telemetry_retry_services_locked();assert(telemetry_router_leave());
   unsigned starts=ota_inits+underglow_inits+registrations;
   if(bit){assert(g_telemetry_service_faults==bit&&ota_inits+underglow_inits+registrations==starts);
    clock_ms+=TELEMETRY_SERVICE_RETRY_MS;
    assert(telemetry_router_enter());telemetry_retry_services_locked();assert(telemetry_router_leave());
    assert(ota_inits+underglow_inits+registrations>starts||bit==TELEMETRY_SERVICE_TIMESYNC);}
   assert(g_telemetry_service_faults==0&&g_cache_router==g_router.r&&frees==0&&allocations==1);
   assert(ota_inits==1+(bit==TELEMETRY_SERVICE_OTA)&&underglow_inits==1+(bit==TELEMETRY_SERVICE_UNDERGLOW));
   for(unsigned i=0;i<allocations;i++)if(allocations_list[i]==g_router.r)__real_seds_router_free(allocations_list[i]);
   printf("PASS %s relay-first service fault=%u\n",argv[2],(unsigned)bit);
   pthread_mutex_destroy(&g_telemetry_mutex.mutex);pthread_mutexattr_destroy(&attr);return 0;
  }
  fail_stage=(stage<=3||stage>=6)?stage:0;fail_enable=stage==4;fail_callback=stage==5;
  assert(init_telemetry_router()!=SEDS_OK&&g_router.r==NULL&&g_router.created==0&&g_cache_router==NULL);
  unsigned count=allocations,freed=frees,w=writes;assert(freed==count&&w==0);
  for(unsigned i=0;i<20;i++)assert(init_telemetry_router()==SEDS_IO&&allocations==count&&frees==freed&&writes==w);
  clock_ms+=499;assert(init_telemetry_router()==SEDS_IO&&allocations==count);
  fail_stage=0;fail_enable=fail_callback=false;clock_ms++;assert(init_telemetry_router()==SEDS_OK&&g_router.created==1&&g_cache_router==g_router.r);
  assert(allocations==count+1&&frees==freed&&g_router_retry_after_ms==0&&g_telemetry_init_error_code==TELEMETRY_INIT_OK);
 }
 for(unsigned i=0;i<allocations;i++)if(allocations_list[i]==g_router.r)__real_seds_router_free(allocations_list[i]);
 printf("PASS %s actual_constructor+cache+RX wrapper realSDK allocations=%u frees=%u callbacks=%u\n",argv[2],allocations,frees,app_callbacks);
 pthread_mutex_destroy(&g_telemetry_mutex.mutex);pthread_mutexattr_destroy(&attr);return 0;
}
