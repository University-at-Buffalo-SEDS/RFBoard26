"""Compile the actual cache callback and fault-inject its NVM/apply boundaries.

Once a station FLIGHT_STATE write is applied it is acknowledged (SEDS_OK)
even when flash or the OTA owner is unavailable: an error withholds the
end-to-end receipt, so the station's ordered FLIGHT_STATE stream stalls and
newer writes, Aborted among them, queue behind it. Durability is retried
from flight_state_cache_poll at most once per second.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
HEADERS = {
    "ota_stream.h": '#pragma once\n#include <stdbool.h>\n#include <stdint.h>\n/* This fixture isolates cache/router/persistence semantics. The OTA owner\n * is a fault-injected permit; real OTA admission is tested by\n * test_ota_lifecycle_native and the integrated strict recovery fixture. */\nbool ota_stream_state_begin(uint8_t phase);\nvoid ota_stream_state_end(void);\nstatic inline void ota_stream_recovery_complete(uint32_t epoch){(void)epoch;}\n',
    "board_recovery_transport.h": '#pragma once\n#include "sedsnet.h"\nuint8_t board_recovery_managed_update_allowed(const SedsPacketView*);\n',
    "sedsnet.h": r'''
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
typedef int SedsResult;
typedef int SedsRouter;
typedef struct { uint32_t ty; const uint8_t *payload; size_t payload_len; uint64_t timestamp; const char *sender; size_t sender_len; } SedsPacketView;
#define SEDS_OK 0
#define SEDS_HANDLER_ERROR 1
#define SEDS_BAD_ARG 2
SedsResult seds_router_enable_network_variable(SedsRouter*,uint32_t,bool,bool);
SedsResult seds_router_on_network_variable_update(SedsRouter*,uint32_t,SedsResult(*)(const SedsPacketView*,void*),void*);
SedsResult seds_router_request_managed_variable(SedsRouter*,uint32_t);
void seds_router_disable_managed_variable(SedsRouter*,uint32_t);
SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*,uint32_t,bool*);
''',
    "sedsnet_config.h": "#pragma once\n#define SEDS_DT_FLIGHT_STATE 444U\n",
    "main.h": "#pragma once\n#include <stdint.h>\nuint32_t HAL_GetTick(void);\n",
    "persistent_store.h": r'''
#pragma once
#include <stddef.h>
#include <stdint.h>
typedef int launchcore_persist_status_t;
#define LAUNCHCORE_PERSIST_OK 0
#define LAUNCHCORE_PERSIST_NOT_FOUND 1
#define LAUNCHCORE_PERSIST_STORAGE_ERROR 2
launchcore_persist_status_t persistent_store_init(void);
launchcore_persist_status_t persistent_store_get(uint32_t,void*,size_t*);
launchcore_persist_status_t persistent_store_set(uint32_t,const void*,size_t);
''',
}
HARNESS = r'''
#include "flight_state_cache.h"
#include "persistent_store.h"
#include "sedsnet_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
volatile uint32_t g_telemetry_discovery_seen;
extern volatile uint32_t g_flight_state_cache_value;
extern volatile uint32_t g_flight_state_cache_restores;
extern volatile uint32_t g_flight_state_cache_writes;
extern volatile uint32_t g_flight_state_cache_updates;
extern volatile uint32_t g_flight_state_cache_errors;
static SedsResult (*callback)(const SedsPacketView*,void*);
static unsigned write_attempts, apply_attempts, requests;
static int init_failed, write_failed, have_stored, ota_busy, ota_owned;
static uint8_t stored, applied;
static uint32_t tick;
static SedsResult apply_result = SEDS_OK;
uint32_t HAL_GetTick(void) { return tick; }
bool ota_stream_state_begin(uint8_t phase) {
 assert(phase<=16U && !ota_owned);
 if(ota_busy)return false;
 ota_owned=1;return true;
}
void ota_stream_state_end(void) { assert(ota_owned);ota_owned=0; }
int persistent_store_init(void) { return init_failed ? LAUNCHCORE_PERSIST_STORAGE_ERROR : LAUNCHCORE_PERSIST_OK; }
int persistent_store_get(uint32_t key,void *value,size_t *size) {
 assert(key==0x46535445u && *size==1);
 if(!have_stored)return LAUNCHCORE_PERSIST_NOT_FOUND;
 *(uint8_t*)value=stored;return LAUNCHCORE_PERSIST_OK;
}
int persistent_store_set(uint32_t key,const void *value,size_t size) {
 assert(key==0x46535445u && size==1);
 assert(applied==*(const uint8_t*)value); /* Preserve apply-before-flash behavior. */
 assert(ota_owned); /* Flash is written only while the OTA owner is held. */
 ++write_attempts;
 if(write_failed)return LAUNCHCORE_PERSIST_STORAGE_ERROR;
 stored=*(const uint8_t*)value;have_stored=1;return LAUNCHCORE_PERSIST_OK;
}
SedsResult flight_state_cache_apply_network_update(const SedsPacketView *packet) {
 ++apply_attempts;
 if(apply_result==SEDS_OK)applied=packet->payload[0];
 return apply_result;
}
static unsigned selective_enable_calls,selective_callback_calls;
SedsResult seds_router_enable_network_variable(SedsRouter *r,uint32_t ty,bool write,bool other) {assert(++selective_enable_calls==1U);
 assert(r && ty==SEDS_DT_FLIGHT_STATE && write && !other);return SEDS_OK;
}
SedsResult seds_router_on_network_variable_update(SedsRouter *r,uint32_t ty,SedsResult(*fn)(const SedsPacketView*,void*),void *user) {assert(++selective_callback_calls==1U);
 assert(r && ty==SEDS_DT_FLIGHT_STATE && fn && !user);callback=fn;return SEDS_OK;
}
SedsResult seds_router_request_managed_variable(SedsRouter *r,uint32_t ty) {
 assert(r && ty==SEDS_DT_FLIGHT_STATE);++requests;return SEDS_OK;
}
/* The actual SDK retirement lifecycle is exercised separately with the
 * public435 archive; this fixture isolates NVM/apply retry semantics. */
void seds_router_disable_managed_variable(SedsRouter *r,uint32_t ty) {(void)r;(void)ty;assert(!"whole-service disable forbidden during selective retirement");}
SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*r,uint32_t ty,bool*out_invalidated){assert(out_invalidated==NULL);
 assert(r && ty==SEDS_DT_FLIGHT_STATE);
return SEDS_OK;}

uint8_t board_recovery_managed_update_allowed(const SedsPacketView *packet) {
 assert(packet);return 1U;
}
/* Each call is a new station write (its own station timestamp). */
static uint64_t write_stamp=1;
static SedsResult update(uint8_t state) {
 SedsPacketView packet={.ty=SEDS_DT_FLIGHT_STATE,.payload=&state,.payload_len=1,.timestamp=write_stamp++,.sender="GS",.sender_len=2};
 assert(callback);SedsResult r=callback(&packet,NULL);assert(!ota_owned);return r;
}
static SedsRouter poll_router;
static void poll_at(uint32_t at) { tick=at;assert(flight_state_cache_poll(&poll_router)==SEDS_OK && !ota_owned); }
int main(int argc,char **argv) {
 assert(argc==2);SedsRouter router=0;
 if(!strcmp(argv[1],"restore")){have_stored=1;stored=13;}
 if(!strcmp(argv[1],"invalid_restore")){have_stored=1;stored=17;}
 if(!strcmp(argv[1],"init_error"))init_failed=1;
 assert(flight_state_cache_init(&router)==SEDS_OK && callback);
 if(!strcmp(argv[1],"changed_retry")){
  assert(update(1)==SEDS_OK && write_attempts==1 && stored==1);
  assert(update(1)==SEDS_OK && write_attempts==1);
  write_failed=1;tick=10000;
  /* Applied and acknowledged; durability is retried, not the receipt. */
  assert(update(0)==SEDS_OK && write_attempts==2 && stored==1);
  assert(applied==0 && flight_state_cache_has_value() && flight_state_cache_value()==0);
  poll_at(10999);assert(write_attempts==2);              /* at most once per second */
  poll_at(11000);assert(write_attempts==3 && stored==1);
  poll_at(11500);assert(write_attempts==3);
  write_failed=0;
  poll_at(11999);assert(write_attempts==3);
  poll_at(12000);assert(write_attempts==4 && stored==0);
  poll_at(20000);assert(write_attempts==4);              /* nothing pending */
  assert(update(0)==SEDS_OK && write_attempts==4);
  assert(g_flight_state_cache_writes==2 && g_flight_state_cache_errors==2 && apply_attempts==4);
 }else if(!strcmp(argv[1],"newer_value_while_pending")){
  write_failed=1;tick=1000;
  assert(update(1)==SEDS_OK && write_attempts==1);
  assert(update(2)==SEDS_OK && write_attempts==2);       /* a newer write still tries now */
  write_failed=0;poll_at(2000);
  assert(write_attempts==3 && stored==2);                 /* the latest applied value */
  write_failed=1;assert(update(2)==SEDS_OK && write_attempts==3);
 }else if(!strcmp(argv[1],"missing_zero")){
  assert(!flight_state_cache_has_value());write_failed=1;tick=1000;
  assert(update(0)==SEDS_OK && write_attempts==1);
  assert(update(0)==SEDS_OK && write_attempts==2);       /* still pending: a retry writes */
  write_failed=0;poll_at(2000);assert(write_attempts==3 && stored==0);
  assert(update(0)==SEDS_OK && write_attempts==3);
 }else if(!strcmp(argv[1],"restore")){
  assert(flight_state_cache_has_value() && flight_state_cache_value()==13 && g_flight_state_cache_restores==1);
  assert(update(13)==SEDS_OK && write_attempts==0);
  write_failed=1;tick=1000;
  assert(update(6)==SEDS_OK && write_attempts==1 && stored==13);
  assert(flight_state_cache_value()==6 && applied==6);
  write_failed=0;poll_at(2000);assert(write_attempts==2 && stored==6);
 }else if(!strcmp(argv[1],"invalid_restore")){
  assert(!flight_state_cache_has_value() && g_flight_state_cache_errors==1);
  assert(update(0)==SEDS_OK && write_attempts==1 && stored==0);
 }else if(!strcmp(argv[1],"apply_error")){
  assert(update(1)==SEDS_OK && write_attempts==1);apply_result=SEDS_BAD_ARG;
  assert(update(2)==SEDS_BAD_ARG && write_attempts==1 && flight_state_cache_value()==1);
  apply_result=SEDS_OK;assert(update(2)==SEDS_OK && write_attempts==2 && stored==2);
 }else if(!strcmp(argv[1],"init_error")){
  tick=1000;
  assert(update(1)==SEDS_OK && write_attempts==0);
  assert(applied==1 && flight_state_cache_value()==1 && g_flight_state_cache_errors==2);
  poll_at(2000);assert(write_attempts==0 && g_flight_state_cache_errors==3);
  init_failed=0;poll_at(2500);assert(write_attempts==0);
  poll_at(3000);assert(write_attempts==1 && stored==1);
  assert(update(1)==SEDS_OK && write_attempts==1);
 }else if(!strcmp(argv[1],"ota_busy")){
  ota_busy=1;tick=1000;
  /* The OTA owner holds flash: apply and acknowledge, never write now. */
  assert(update(6)==SEDS_OK && write_attempts==0 && applied==6 && flight_state_cache_value()==6);
  poll_at(2000);assert(write_attempts==0 && g_flight_state_cache_errors==2);
  ota_busy=0;poll_at(2999);assert(write_attempts==0);
  poll_at(3000);assert(write_attempts==1 && stored==6);
 }else if(!strcmp(argv[1],"invalid_packet")){
  uint8_t state=17;SedsPacketView packet={.ty=SEDS_DT_FLIGHT_STATE,.payload=&state,.payload_len=1,.timestamp=write_stamp++,.sender="GS",.sender_len=2};
  assert(callback(NULL,NULL)==SEDS_HANDLER_ERROR);
  /* A malformed FLIGHT_STATE never succeeds on retry: acknowledged, unapplied. */
  assert(callback(&packet,NULL)==SEDS_OK);
  state=1;packet.payload_len=0;assert(callback(&packet,NULL)==SEDS_OK);
  packet.payload_len=2;assert(callback(&packet,NULL)==SEDS_OK);
  packet.payload_len=1;packet.ty=0;assert(callback(&packet,NULL)==SEDS_HANDLER_ERROR);
  packet.ty=SEDS_DT_FLIGHT_STATE;packet.payload=NULL;assert(callback(&packet,NULL)==SEDS_OK);
  assert(write_attempts==0 && apply_attempts==0 && !flight_state_cache_has_value());
 }else if(!strcmp(argv[1],"poll")){
  tick=500;assert(flight_state_cache_poll(&router)==SEDS_OK && requests==0);
  g_telemetry_discovery_seen=1;tick=499;assert(flight_state_cache_poll(&router)==SEDS_OK && requests==0);
  tick=500;assert(flight_state_cache_poll(&router)==SEDS_OK && requests==1);
  /* The station does not answer a GET for a state it has no authority for,
   * so unanswered GETs back off 0.5 -> 1 -> 2 -> 4 s instead of 2 Hz forever. */
  tick=1000;assert(flight_state_cache_poll(&router)==SEDS_OK && requests==1);
  tick=1499;assert(flight_state_cache_poll(&router)==SEDS_OK && requests==1);
  tick=1500;assert(flight_state_cache_poll(&router)==SEDS_OK && requests==2);
  tick=3499;assert(flight_state_cache_poll(&router)==SEDS_OK && requests==2);
  tick=3500;assert(flight_state_cache_poll(&router)==SEDS_OK && requests==3);
  tick=7499;assert(flight_state_cache_poll(&router)==SEDS_OK && requests==3);
  tick=7500;assert(flight_state_cache_poll(&router)==SEDS_OK && requests==4);
  tick=11500;assert(flight_state_cache_poll(&router)==SEDS_OK && requests==5);
  tick=15499;assert(flight_state_cache_poll(&router)==SEDS_OK && requests==5);
  tick=15500;assert(flight_state_cache_poll(&router)==SEDS_OK && requests==6);
  write_failed=1;assert(update(1)==SEDS_OK);
  /* A seen value stops the GET; only the durability retry continues. */
  tick=17000;assert(flight_state_cache_poll(&router)==SEDS_OK && requests==6 && write_attempts==2);
 }else assert(!"unknown test case");
 printf("flight-state-cache %s passed\n",argv[1]);return 0;
}
'''

class FlightStatePersistenceRetryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="flight-state-cache-native-")
        root = Path(cls.temp.name)
        for name, text in HEADERS.items():
            (root / name).write_text(text)
        (root / "harness.c").write_text(HARNESS)
        cls.executable = root / "test"
        subprocess.run(["cc", "-DTELEMETRY_ENABLED=1", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-fsanitize=address,undefined", "-I", str(root), "-I", str(ROOT / "Core/Inc"),
                        str(ROOT / "Core/Src/flight_state_cache.c"), str(root / "harness.c"),
                        "-o", str(cls.executable)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def run_case(self, name):
        subprocess.run([str(self.executable), name], check=True)

    def test_failed_write_is_acknowledged_and_retried_once_per_second(self):
        self.run_case("changed_retry")

    def test_missing_default_zero_is_not_false_durability(self):
        self.run_case("missing_zero")

    def test_successful_restore_avoids_wear_but_failed_new_value_retries(self):
        self.run_case("restore")

    def test_invalid_restore_requires_a_real_successful_write(self):
        self.run_case("invalid_restore")

    def test_apply_failure_preserves_existing_order_and_error(self):
        self.run_case("apply_error")

    def test_storage_initialization_failure_is_retried_after_the_receipt(self):
        self.run_case("init_error")

    def test_busy_ota_owner_defers_flash_but_not_the_receipt(self):
        self.run_case("ota_busy")

    def test_newer_value_while_pending_persists_the_latest(self):
        self.run_case("newer_value_while_pending")

    def test_invalid_packet_never_applies_or_writes(self):
        self.run_case("invalid_packet")

    def test_unanswered_get_starts_at_500ms_after_discovery_and_backs_off(self):
        self.run_case("poll")

if __name__ == "__main__":
    unittest.main()
