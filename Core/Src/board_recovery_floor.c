#include "board_recovery_floor.h"
#include "persistent_store.h"
#include <stddef.h>
#define BOARD_RECOVERY_FLOOR_KEY UINT32_C(0x4252464c)
/* 0=not loaded, 1=never recovered, 2=verified floor, 3=fail closed. */
static uint8_t floor_state;
static uint64_t floor_timestamp;
/* Canonical source time is monotonic across every issuer. Burn it BEFORE
 * uncertain writes; changing issuer cannot lower verified/attempted floors.
 * UTC rollback remains blocked until canonical time passes this bound. */
static uint64_t attempted_timestamp;
static uint64_t decode_u64(const uint8_t *bytes) {
 uint64_t value=0U; for (unsigned i=0U;i<8U;i++) value|=(uint64_t)bytes[i]<<(8U*i); return value;
}
static void encode_u64(uint8_t *bytes,uint64_t value) {
 for (unsigned i=0U;i<8U;i++) bytes[i]=(uint8_t)(value>>(8U*i));
}
static void load_floor(void) {
 if (floor_state!=0U) return;
 floor_state=3U;
 if (persistent_store_init()!=LAUNCHCORE_PERSIST_OK) return;
 uint8_t wire[16]={0U}; size_t length=sizeof wire;
 launchcore_persist_status_t result=persistent_store_get(BOARD_RECOVERY_FLOOR_KEY,wire,&length);
 if (result==LAUNCHCORE_PERSIST_NOT_FOUND) { floor_state=1U; return; }
 if (result!=LAUNCHCORE_PERSIST_OK || length!=sizeof wire ||
     decode_u64(wire)==0U || decode_u64(wire+8U)==0U) return;
 floor_timestamp=decode_u64(wire+8U); floor_state=2U;
}
uint8_t board_recovery_floor_active(void) { load_floor(); return floor_state==2U; }
uint8_t board_recovery_floor_allows_recovery_time(uint64_t issuer_session,uint64_t source_time_ms) {
 load_floor();
 return issuer_session!=0U && source_time_ms!=0U &&
        source_time_ms>floor_timestamp && source_time_ms>attempted_timestamp;
}
uint8_t board_recovery_floor_is_healthy(void) { load_floor(); return floor_state!=3U; }
uint8_t board_recovery_command_time_allowed(uint64_t canonical_time_ms) {
 load_floor();
 if (floor_state==1U) return 1U;
 return floor_state==2U && canonical_time_ms>floor_timestamp;
}
uint8_t board_recovery_floor_commit(uint64_t issuer_session,uint64_t source_time_ms) {
 if (!board_recovery_floor_allows_recovery_time(issuer_session,source_time_ms)) return 0U;
 /* A failed attempted recovery cannot leave an unverified relaxed RAM floor. */
 floor_state=3U;
 attempted_timestamp=source_time_ms;
 if (persistent_store_init()!=LAUNCHCORE_PERSIST_OK) return 0U;
 uint8_t wire[16]; encode_u64(wire,issuer_session); encode_u64(wire+8U,source_time_ms);
 if (persistent_store_set(BOARD_RECOVERY_FLOOR_KEY,wire,sizeof wire)!=LAUNCHCORE_PERSIST_OK) return 0U;
 uint8_t verified[16]={0U}; size_t length=sizeof verified;
 if (persistent_store_get(BOARD_RECOVERY_FLOOR_KEY,verified,&length)!=LAUNCHCORE_PERSIST_OK ||
     length!=sizeof verified || decode_u64(verified)!=issuer_session ||
     decode_u64(verified+8U)!=source_time_ms) return 0U;
 floor_timestamp=source_time_ms; floor_state=2U; return 1U;
}
