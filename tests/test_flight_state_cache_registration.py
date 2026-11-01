"""Actual cache-owner partial-registration failures must never confirm recovery."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
HEADERS = {'sedsnet.h': '#pragma once\n'
              '#include <stdint.h>\n'
              '#include <stddef.h>\n'
              '#include <stdbool.h>\n'
              'typedef int SedsResult;\n'
              'typedef int SedsRouter;\n'
              'typedef struct { unsigned ty; const uint8_t *payload; size_t payload_len; '
              'uint64_t timestamp; } SedsPacketView;\n'
              '#define SEDS_OK 0\n'
              '#define SEDS_BAD_ARG -10\n'
              '#define SEDS_HANDLER_ERROR -9\n'
              'SedsResult '
              'seds_router_enable_network_variable(SedsRouter*,unsigned,bool,bool);\n'
              'SedsResult '
              'seds_router_on_network_variable_update(SedsRouter*,unsigned,SedsResult(*)(const '
              'SedsPacketView*,void*),void*);\n'
              'SedsResult seds_router_request_managed_variable(SedsRouter*,unsigned);\n'
              'void seds_router_disable_managed_variable(SedsRouter*,unsigned);\n'
              'SedsResult '
              'seds_router_invalidate_network_variable_cache(SedsRouter*,unsigned,bool*);\n',
 'sedsnet_config.h': '#pragma once\n#define SEDS_DT_FLIGHT_STATE 444U\n',
 'main.h': '#pragma once\n#include <stdint.h>\nuint32_t HAL_GetTick(void);\n',
 'board_recovery_transport.h': '#pragma once\n'
                               '#include "sedsnet.h"\n'
                               'uint8_t board_recovery_managed_update_allowed(const '
                               'SedsPacketView*);\n',
 'persistent_store.h': '#pragma once\n'
                       '#include <stddef.h>\n'
                       '#include <stdint.h>\n'
                       'typedef int launchcore_persist_status_t;\n'
                       '#define LAUNCHCORE_PERSIST_OK 0\n'
                       '#define LAUNCHCORE_PERSIST_NOT_FOUND 1\n'
                       '#define LAUNCHCORE_PERSIST_STORAGE_ERROR 2\n'
                       'int persistent_store_init(void);\n'
                       'int persistent_store_get(uint32_t,void*,size_t*);\n'
                       'int persistent_store_set(uint32_t,const void*,size_t);\n'}
HARNESS = r'''
#include "flight_state_cache.h"
#include "persistent_store.h"
#include "sedsnet_config.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
volatile uint32_t g_telemetry_discovery_seen;
static unsigned writes,invalidations,enables,registrations;
static int fail_enable,fail_callback;
static uint8_t stored=5;
uint32_t HAL_GetTick(void){return 1000;}
int persistent_store_init(void){return 0;}
int persistent_store_get(uint32_t key,void*p,size_t*n){assert(key==0x46535445U&&*n==1);*(uint8_t*)p=stored;return 0;}
int persistent_store_set(uint32_t key,const void*p,size_t n){assert(key==0x46535445U&&n==1);writes++;stored=*(const uint8_t*)p;return 0;}
uint8_t board_recovery_managed_update_allowed(const SedsPacketView*p){assert(p);return 1;}
SedsResult seds_router_enable_network_variable(SedsRouter*r,unsigned ty,bool rd,bool wr){assert(r&&ty==SEDS_DT_FLIGHT_STATE&&rd&&!wr);enables++;return fail_enable?SEDS_HANDLER_ERROR:SEDS_OK;}
SedsResult seds_router_on_network_variable_update(SedsRouter*r,unsigned ty,SedsResult(*cb)(const SedsPacketView*,void*),void*u){assert(r&&ty==SEDS_DT_FLIGHT_STATE&&cb&&!u);registrations++;return fail_callback?SEDS_HANDLER_ERROR:SEDS_OK;}
void seds_router_disable_managed_variable(SedsRouter*r,unsigned ty){(void)r;(void)ty;assert(!"selective retirement must not disable service");}
SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*r,unsigned ty,bool*out){assert(r&&ty==SEDS_DT_FLIGHT_STATE&&!out);invalidations++;return SEDS_OK;}
SedsResult seds_router_request_managed_variable(SedsRouter*r,unsigned ty){assert(r&&ty==SEDS_DT_FLIGHT_STATE);return SEDS_OK;}
/* Test the actual static readiness guard as well as its public reset caller. */
#include CACHE_SOURCE
int main(int argc,char**argv){
 assert(argc==2);SedsRouter router=0;
 assert(clear_managed_cache()==SEDS_BAD_ARG&&invalidations==0);
 fail_enable=!strcmp(argv[1],"enable_failure");fail_callback=!strcmp(argv[1],"callback_failure");
 if(fail_enable||fail_callback){
  assert(flight_state_cache_init(&router)==SEDS_HANDLER_ERROR&&g_cache_router==NULL);
  assert(writes==0&&invalidations==0&&flight_state_cache_value()==5);
  assert(clear_managed_cache()==SEDS_BAD_ARG);
  assert(flight_state_cache_reset_startup()==-9&&writes==1&&stored==0&&invalidations==0);
  assert(flight_state_cache_value()==5); /* No falsely confirmed volatile Startup. */
  fail_enable=fail_callback=0;
 }
 assert(flight_state_cache_init(&router)==SEDS_OK&&g_cache_router==&router);
 unsigned w=writes,e=enables,c=registrations;
 assert(flight_state_cache_init(NULL)==SEDS_BAD_ARG&&g_cache_router==&router);
 assert(enables==e&&registrations==c&&writes==w);
 assert(clear_managed_cache()==SEDS_OK&&invalidations==1); /* healthy absent latest */
 assert(flight_state_cache_reset_startup()==0&&writes==w+1&&invalidations==2);
 puts(argv[1]);return 0;
}
'''
class CacheRegistrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory(prefix="cache-registration-")
        cls.path=Path(cls.tmp.name)
        for name,content in HEADERS.items(): (cls.path/name).write_text(content)
        (cls.path/'harness.c').write_text(HARNESS)
        cls.exe=cls.path/'registration'
        subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-DTELEMETRY_ENABLED=1',
                        '-DCACHE_SOURCE="'+str(ROOT/'Core/Src/flight_state_cache.c')+'"',
                        '-I'+str(cls.path),'-I'+str(ROOT/'Core/Inc'),str(cls.path/'harness.c'),'-o',str(cls.exe)],check=True)
    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()
    def run_case(self,name): subprocess.run([str(self.exe),name],check=True,capture_output=True,text=True)
    def test_enable_failure_requires_explicit_successful_init(self): self.run_case('enable_failure')
    def test_callback_failure_requires_explicit_successful_init(self): self.run_case('callback_failure')
    def test_healthy_empty_and_invalid_null_preserve_readiness(self): self.run_case('healthy')
if __name__=='__main__': unittest.main()
