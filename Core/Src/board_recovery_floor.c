#include "persistent_store_health.h"
#include "board_recovery_floor.h"
#include "persistent_store.h"
#include <stddef.h>
#include <stdatomic.h>
#define BOARD_RECOVERY_FLOOR_KEY UINT32_C(0x4252464c)
/* A checked, nonblocking owner protects every cache field, including64-bit
 * values on CortexM4. No caller waits for this owner or calls SDK while held. */
static atomic_flag floor_owner = ATOMIC_FLAG_INIT;
static uint8_t floor_state; /*0 unloaded,1 healthy/no prior recovery,2 verified,3 failed */
static uint64_t floor_timestamp;
static uint64_t attempted_timestamp;
static uint8_t enter_floor(void) {
 return !atomic_flag_test_and_set_explicit(&floor_owner,memory_order_acquire);
}
static void leave_floor(void) {
 atomic_flag_clear_explicit(&floor_owner,memory_order_release);
}
static uint64_t decode_u64(const uint8_t *bytes) {
 uint64_t value=0U;for(unsigned i=0U;i<8U;i++)value|=(uint64_t)bytes[i]<<(8U*i);return value;
}
static void encode_u64(uint8_t *bytes,uint64_t value) {
 for(unsigned i=0U;i<8U;i++)bytes[i]=(uint8_t)(value>>(8U*i));
}
/* Internal helpers require floor_owner; never recursively enter public APIs. */
static void load_floor_locked(void) {
 if(floor_state!=0U)return;
 floor_state=3U;
 if(persistent_store_init()!=LAUNCHCORE_PERSIST_OK)return;
 uint8_t wire[16]={0U};size_t length=sizeof wire;
 launchcore_persist_status_t result=persistent_store_get(BOARD_RECOVERY_FLOOR_KEY,wire,&length);
 /* Only strict healthy ABSENT keeps facade readiness; deleted/error does not. */
 if(result==LAUNCHCORE_PERSIST_NOT_FOUND&&persistent_store_controls_ready()){floor_state=1U;return;}
 if(result!=LAUNCHCORE_PERSIST_OK||length!=sizeof wire||decode_u64(wire)==0U||decode_u64(wire+8U)==0U)return;
 floor_timestamp=decode_u64(wire+8U);floor_state=2U;
}
static uint8_t allows_locked(uint64_t issuer,uint64_t source) {
 load_floor_locked();
 return issuer!=0U&&source!=0U&&source>floor_timestamp&&source>attempted_timestamp;
}
uint8_t board_recovery_floor_active(void) {
 /* Busy is conservatively active: the caller MUST retain freshness checks. */
 if(!enter_floor())return 1U;
 load_floor_locked();uint8_t active=floor_state==2U;leave_floor();return active;
}
uint8_t board_recovery_floor_allows_recovery_time(uint64_t issuer,uint64_t source) {
 if(!enter_floor())return 0U;
 uint8_t allowed=allows_locked(issuer,source);leave_floor();return allowed;
}
uint8_t board_recovery_floor_is_healthy(void) {
 if(!enter_floor())return 0U;
 load_floor_locked();uint8_t healthy=floor_state!=3U;leave_floor();return healthy;
}
uint8_t board_recovery_command_time_allowed(uint64_t source) {
 if(!enter_floor())return 0U;
 load_floor_locked();uint8_t allowed=floor_state==1U||(floor_state==2U&&source>floor_timestamp);
 leave_floor();return allowed;
}
uint8_t board_recovery_floor_commit(uint64_t issuer,uint64_t source) {
 if(!enter_floor())return 0U;
 if(!allows_locked(issuer,source)){leave_floor();return 0U;}
 /* Burn original time before uncertain IO. Checked owner contention may refuse
  * a request but never lowers the verified or attempted admission cutoff. */
 floor_state=3U;attempted_timestamp=source;
 uint8_t success=0U;
 if(persistent_store_init()==LAUNCHCORE_PERSIST_OK) {
  uint8_t wire[16];encode_u64(wire,issuer);encode_u64(wire+8U,source);
  uint8_t stored=persistent_store_set(BOARD_RECOVERY_FLOOR_KEY,wire,sizeof wire)==LAUNCHCORE_PERSIST_OK;
  if(stored) {
   uint8_t verified[16]={0U};size_t length=sizeof verified;
   if(persistent_store_get(BOARD_RECOVERY_FLOOR_KEY,verified,&length)==LAUNCHCORE_PERSIST_OK&&length==sizeof verified&&decode_u64(verified)==issuer&&decode_u64(verified+8U)==source) {
    floor_timestamp=source;floor_state=2U;success=1U;
   }
  }
 }
 leave_floor();return success;
}
uint8_t board_recovery_floor_read_cutoff(uint64_t *cutoff) {
 if(cutoff==NULL||!enter_floor())return 0U;
 load_floor_locked();uint8_t healthy=floor_state!=3U;
 if(healthy)*cutoff=floor_state==2U?floor_timestamp:0U;
 leave_floor();return healthy;
}
uint8_t board_recovery_floor_accept_bootstrap(uint64_t source) {
 if(!source||source==UINT64_MAX)return 0U;
 if(!enter_floor())return 0U;
 if(source>floor_timestamp)floor_timestamp=source;
 if(source>attempted_timestamp)attempted_timestamp=source;
 floor_state=2U;leave_floor();return 1U;
}
uint8_t board_recovery_floor_note_bootstrap_attempt(uint64_t source) {
 if(!source||source==UINT64_MAX)return 0U;
 if(!enter_floor())return 0U;
 if(source>attempted_timestamp)attempted_timestamp=source;
 leave_floor();return 1U;
}

uint8_t board_recovery_floor_prime_boot(void){if(!enter_floor())return 0U;load_floor_locked();uint8_t healthy=floor_state==1U||floor_state==2U;leave_floor();return healthy;}
uint8_t board_recovery_floor_cached_health(void){if(!enter_floor())return 0U;uint8_t healthy=floor_state==1U||floor_state==2U;leave_floor();return healthy;}

/* QUERY-only accessor: never invokes the lazy loader or touches storage. */
uint8_t board_recovery_floor_cached_cutoff(uint64_t*cutoff){
 if(!cutoff||!enter_floor())return 0;
 uint8_t known=floor_state!=0;
 if(known){*cutoff=floor_timestamp>attempted_timestamp?floor_timestamp:attempted_timestamp;}
 leave_floor();return known;
}
