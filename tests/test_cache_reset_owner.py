"""Exercise the actual cache/reset owner with NVM and managed-cache fault injection."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]
HEADERS = {
"sedsnet.h": r'''#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
typedef int SedsResult;
typedef int SedsRouter;
typedef struct { unsigned ty; const uint8_t *payload; size_t payload_len; uint64_t timestamp; } SedsPacketView;
#define SEDS_OK 0
#define SEDS_BAD_ARG -10
#define SEDS_HANDLER_ERROR -9
SedsResult seds_router_enable_network_variable(SedsRouter*,unsigned,bool,bool);
SedsResult seds_router_on_network_variable_update(SedsRouter*,unsigned,SedsResult(*)(const SedsPacketView*,void*),void*);
SedsResult seds_router_request_managed_variable(SedsRouter*,unsigned);
void seds_router_disable_managed_variable(SedsRouter*,unsigned);
SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*,unsigned,bool*);
''',
"sedsnet_config.h": '#pragma once\n#define SEDS_DT_FLIGHT_STATE 444U\n',
"main.h": '#pragma once\n#include <stdint.h>\nuint32_t HAL_GetTick(void);\n',
"board_recovery_transport.h": '#pragma once\n#include "sedsnet.h"\nuint8_t board_recovery_managed_update_allowed(const SedsPacketView*);\n',
"persistent_store.h": r'''#pragma once
#include <stddef.h>
#include <stdint.h>
typedef int launchcore_persist_status_t;
#define LAUNCHCORE_PERSIST_OK 0
#define LAUNCHCORE_PERSIST_NOT_FOUND 1
#define LAUNCHCORE_PERSIST_STORAGE_ERROR 2
int persistent_store_init(void);
int persistent_store_get(uint32_t,void*,size_t*);
int persistent_store_set(uint32_t,const void*,size_t);
'''}
HARNESS = r'''
#include "flight_state_cache.h"
#include "board_recovery.h"
#include "persistent_store.h"
#include "sedsnet_config.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
volatile uint32_t g_telemetry_discovery_seen;
static SedsResult (*callback)(const SedsPacketView*,void*);
static unsigned writes,clears,requests;
static uint8_t stored=5,have_stored=1,allow_update=1;
static int fail_write,fail_read,wrong_read,fail_invalidate,fail_apply;
static unsigned tick;
uint32_t HAL_GetTick(void){return tick;}
SedsResult flight_state_cache_apply_network_update(const SedsPacketView *packet){assert(packet);return fail_apply?-14:0;}
int persistent_store_init(void){return 0;}
int persistent_store_set(uint32_t key,const void *p,size_t n){
 assert(key==0x46535445u&&n==1);++writes;
 if(fail_write)return 2;
 stored=*(const uint8_t*)p;have_stored=1;return 0;
}
int persistent_store_get(uint32_t key,void *p,size_t *n){
 assert(key==0x46535445u&&*n==1);
 if(fail_read)return 2;
 if(!have_stored)return 1;
 *(uint8_t*)p=wrong_read?15:stored;return 0;
}
static unsigned selective_enable_calls,selective_callback_calls;
SedsResult seds_router_enable_network_variable(SedsRouter *r,unsigned ty,bool rd,bool wr){assert(++selective_enable_calls==1U);
 assert(r&&ty==SEDS_DT_FLIGHT_STATE&&rd&&!wr);return SEDS_OK;
}
SedsResult seds_router_on_network_variable_update(SedsRouter *r,unsigned ty,SedsResult(*fn)(const SedsPacketView*,void*),void *ctx){assert(++selective_callback_calls==1U);
 assert(r&&ty==SEDS_DT_FLIGHT_STATE&&fn&&!ctx);callback=fn;return 0;
}
void seds_router_disable_managed_variable(SedsRouter *r,unsigned ty){(void)r;(void)ty;assert(!"whole-service disable forbidden during selective retirement");}
SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*r,unsigned ty,bool*out_invalidated){assert(out_invalidated==NULL);if(fail_invalidate)return SEDS_HANDLER_ERROR;assert(r&&ty==SEDS_DT_FLIGHT_STATE);++clears;return SEDS_OK;}

SedsResult seds_router_request_managed_variable(SedsRouter *r,unsigned ty){assert(r&&ty==SEDS_DT_FLIGHT_STATE);++requests;return 0;}
uint8_t board_recovery_managed_update_allowed(const SedsPacketView *p){assert(p);return allow_update;}
static SedsResult update(uint8_t state){SedsPacketView p={SEDS_DT_FLIGHT_STATE,&state,1,100};assert(callback);return callback(&p,NULL);}
int main(int argc,char **argv){
 assert(argc==2);SedsRouter router=0;BoardRecoverySnapshot out;
 assert(flight_state_cache_init(&router)==0);
 assert(flight_state_cache_has_value()&&flight_state_cache_value()==5);
 assert(board_recovery_snapshot(&out)==0&&out.actual_state==5&&out.abort_epoch==1);
 assert((out.flags&(BOARD_RECOVERY_STATE_VALID|BOARD_RECOVERY_SUPPORTED))==(BOARD_RECOVERY_STATE_VALID|BOARD_RECOVERY_SUPPORTED));
 assert(!(out.flags&(BOARD_RECOVERY_OUTPUTS_VALID|BOARD_RECOVERY_SAFE_OUTPUTS)));
 if(!strcmp(argv[1],"success")){
  assert(board_recovery_apply_safe(0,1,&out)==0&&stored==0&&out.actual_state==0&&out.abort_epoch==2);
  assert(writes==1&&clears==1&&callback);
  assert(board_recovery_apply_safe(0,1,&out)==0&&writes==1&&clears==1&&out.abort_epoch==2);
  assert(board_recovery_apply_safe(1,2,&out)!=0&&writes==1);
  assert(board_recovery_apply_safe(0,0,&out)!=0&&writes==1);
  assert(board_recovery_apply_safe(0,UINT32_MAX,&out)!=0&&writes==1);
  g_telemetry_discovery_seen=1;tick=499;assert(flight_state_cache_poll(&router)==0&&requests==0);
  tick=500;assert(flight_state_cache_poll(&router)==0&&requests==1);
 }else if(!strcmp(argv[1],"write_failure")){
  fail_write=1;assert(board_recovery_apply_safe(0,1,&out)!=0&&out.actual_state==5&&out.abort_epoch==1&&clears==0);
  fail_write=0;assert(board_recovery_apply_safe(0,1,&out)==0&&out.actual_state==0&&out.abort_epoch==2);
 }else if(!strcmp(argv[1],"read_failure")){
  fail_read=1;assert(board_recovery_apply_safe(0,1,&out)!=0&&out.actual_state==5&&out.abort_epoch==1&&clears==0);
  fail_read=0;assert(board_recovery_apply_safe(0,1,&out)==0&&out.actual_state==0);
 }else if(!strcmp(argv[1],"read_mismatch")){
  wrong_read=1;assert(board_recovery_apply_safe(0,1,&out)!=0&&out.actual_state==5&&out.abort_epoch==1&&clears==0);
  wrong_read=0;assert(board_recovery_apply_safe(0,1,&out)==0&&out.actual_state==0);
 }else if(!strcmp(argv[1],"service_failure")){
  fail_invalidate=1;assert(board_recovery_apply_safe(0,1,&out)!=0&&out.actual_state==5&&out.abort_epoch==1);
  fail_invalidate=0;assert(board_recovery_apply_safe(0,1,&out)==0&&out.actual_state==0&&out.abort_epoch==2);
 }else if(!strcmp(argv[1],"stale_retirement")){
  assert(board_recovery_apply_safe(0,1,&out)==0);unsigned w=writes,c=clears;
  allow_update=0;assert(update(3)==-9&&flight_state_cache_value()==0&&stored==0&&writes==w&&clears==c+1&&callback);
  allow_update=1;assert(update(1)==0&&flight_state_cache_value()==1&&stored==1);
  assert(board_recovery_apply_safe(0,1,&out)!=0&&writes==w+1);
 }else if(!strcmp(argv[1],"ordinary_retry")){
  fail_write=1;assert(update(0)==-9&&flight_state_cache_value()==0);
  assert(update(0)==-9&&writes==2);fail_write=0;
  assert(update(0)==0&&writes==3&&stored==0);assert(update(0)==0&&writes==3&&clears==2); /* Both failed persistence owners retire SDK cache; volatile apply/retry counts unchanged. */
 }else if(!strcmp(argv[1],"apply_failure")){
  fail_apply=1;assert(update(3)==-14&&flight_state_cache_value()==5&&stored==5&&writes==0&&clears==1&&callback);
  fail_apply=0;assert(update(3)==0&&flight_state_cache_value()==3&&stored==3&&writes==1&&clears==1);
 }else if(!strcmp(argv[1],"malformed_retirement")){
  assert(update(255)==-9&&flight_state_cache_value()==5&&stored==5&&writes==0&&clears==1&&callback);
 }else assert(!"unknown case");
 puts(argv[1]);return 0;
}
'''
class ResetOwnerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory(prefix="cache-reset-owner-")
        p=Path(cls.tmp.name)
        for name,source in HEADERS.items():(p/name).write_text(source)
        (p/'harness.c').write_text(HARNESS)
        cls.exe=p/'test'
        subprocess.run(['cc','-DTELEMETRY_ENABLED=1','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(p),'-I',str(ROOT/'Core/Inc'),str(ROOT/'Core/Src/flight_state_cache.c'),str(ROOT/'Core/Src/board_recovery_query.c'),str(p/'harness.c'),'-o',str(cls.exe)],check=True)
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def test_actual_owner_success_and_idempotency(self):self.run_case('success')
    def test_failed_write_preserves_applied_state_and_epoch(self):self.run_case('write_failure')
    def test_failed_readback_preserves_applied_state_and_epoch(self):self.run_case('read_failure')
    def test_wrong_readback_preserves_applied_state_and_epoch(self):self.run_case('read_mismatch')
    def test_failed_cache_retirement_never_claims_reset(self):self.run_case('service_failure')
    def test_rejected_managed_update_retires_sdk_cache_and_keeps_applied_state(self):self.run_case('stale_retirement')
    def test_ordinary_failed_persistence_retries_without_extra_cache_resets(self):self.run_case('ordinary_retry')
    def test_failed_apply_retires_sdk_cache_without_mutating_applied_or_persisted_state(self):self.run_case('apply_failure')
    def test_malformed_fstate_retires_sdk_ordering_poison_without_changing_state(self):self.run_case('malformed_retirement')
    def run_case(self,name):subprocess.run([str(self.exe),name],check=True)
