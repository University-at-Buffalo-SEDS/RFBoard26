"""Actual boot-ID storage policy: priming, RAM-only query and safe retry faults."""
import pathlib
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

HEADER = r'''
#ifndef TEST_PERSIST_H
#define TEST_PERSIST_H
#include <stddef.h>
#include <stdint.h>
typedef int launchcore_persist_status_t;
#define LAUNCHCORE_PERSIST_OK 0
#define LAUNCHCORE_PERSIST_NOT_FOUND 1
launchcore_persist_status_t persistent_store_init(void);
launchcore_persist_status_t persistent_store_get(uint32_t,void*,size_t*);
launchcore_persist_status_t persistent_store_set(uint32_t,const void*,size_t);
#endif
'''
DRIVER = r'''
#include "board_recovery_boot.h"
#include "persistent_store.h"
#include <assert.h>
#include <string.h>
static uint64_t durable=41;
static unsigned calls;
static int fault, safe;
uint8_t board_recovery_boot_retry_is_safe(void){return (uint8_t)safe;}
int persistent_store_init(void){++calls;return fault==1?-9:0;}
int persistent_store_get(uint32_t key,void*out,size_t*len){
 assert(key==UINT32_C(0x42524745));++calls;
 if(fault==3)return -9;
 assert(*len>=8);*len=8;
 for(unsigned i=0;i<8;++i)((uint8_t*)out)[i]=(uint8_t)(durable>>(8*i));
 return 0;
}
int persistent_store_set(uint32_t key,const void*in,size_t len){
 assert(key==UINT32_C(0x42524745)&&len==8);++calls;durable=0;
 for(unsigned i=0;i<8;++i)durable|=(uint64_t)((const uint8_t*)in)[i]<<(8*i);
 if(fault==2){fault=3;return -9;}return 0;
}
int main(int argc,char**argv){
 assert(argc==2);
 if(!strcmp(argv[1],"primed")){
  assert(board_recovery_boot_generation()==42);unsigned before=calls;
  for(unsigned i=0;i<100;++i)assert(board_recovery_boot_query_generation()==42);
  assert(calls==before);
 }else if(!strcmp(argv[1],"defer")){
  fault=1;assert(!board_recovery_boot_generation());fault=0;unsigned before=calls;
  assert(!board_recovery_boot_query_generation());assert(calls==before);
  safe=1;for(unsigned i=0;i<100;i++)assert(!board_recovery_boot_query_generation());assert(calls==before);
  /* Direct storage helper boundary only: no retry transport is registered. */
  assert(board_recovery_boot_generation()==42);
 }else if(!strcmp(argv[1],"uncertain")){
  fault=2;assert(!board_recovery_boot_generation());assert(durable==42);
  fault=0;safe=1;unsigned before=calls;assert(!board_recovery_boot_query_generation());assert(calls==before);
  assert(board_recovery_boot_generation()==43);
 }else if(!strcmp(argv[1],"overflow")){
  durable=UINT64_MAX;assert(!board_recovery_boot_generation());safe=1;
  assert(!board_recovery_boot_query_generation());assert(durable==UINT64_MAX);
 }else if(!strcmp(argv[1],"corrupt")){
  durable=0;assert(!board_recovery_boot_generation());safe=1;
  assert(!board_recovery_boot_query_generation());assert(!durable);
 }else assert(0);
 return 0;
}
'''

class BootPrimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="recovery-boot-prime-")
        cls.directory = pathlib.Path(cls.temp.name)
        (cls.directory / "persistent_store.h").write_text(HEADER)
        (cls.directory / "driver.c").write_text(DRIVER)
        # Copy only the actual source to make its include resolve the test NVM
        # boundary; production boot implementation is not extracted or rewritten.
        shutil.copyfile(ROOT / "Core/Src/board_recovery_boot.c", cls.directory / "boot.c")
        cls.binary = cls.directory / "test"
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-I", str(cls.directory), "-I", str(ROOT / "Core/Inc"),
                        str(cls.directory / "boot.c"), str(cls.directory / "driver.c"),
                        "-o", str(cls.binary)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_primed_queries_never_touch_storage(self): self.case("primed")
    def test_failed_prime_queries_never_retry_even_when_owner_safe(self): self.case("defer")
    def test_uncertain_write_burns_generation(self): self.case("uncertain")
    def test_overflow_stays_unknown(self): self.case("overflow")
    def test_corrupt_zero_stays_unknown(self): self.case("corrupt")
    def case(self, name): subprocess.run([str(self.binary), name], check=True)

if __name__ == "__main__": unittest.main()
