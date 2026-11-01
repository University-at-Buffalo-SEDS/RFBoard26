"""Actual boot owner: only verified persistent generations become published IDs."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
HEADER=r'''#pragma once
#include <stdint.h>
#include <stddef.h>
typedef int launchcore_persist_status_t;
#define LAUNCHCORE_PERSIST_OK 0
#define LAUNCHCORE_PERSIST_NOT_FOUND 1
int persistent_store_init(void);
int persistent_store_get(uint32_t,void*,size_t*);
int persistent_store_set(uint32_t,const void*,size_t);
'''
MAIN=r'''
#include "board_recovery_boot.h"
#include "persistent_store.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static uint64_t disk;
static unsigned have,writes,reads,readback_fail,wrong_read,write_error;
static void put(uint8_t*p,uint64_t n){for(unsigned i=0;i<8;i++)p[i]=(uint8_t)(n>>(8*i));}
static uint64_t get(const uint8_t*p){uint64_t n=0;for(unsigned i=0;i<8;i++)n|=(uint64_t)p[i]<<(8*i);return n;}
int persistent_store_init(void){return 0;}
int persistent_store_get(uint32_t key,void*p,size_t*n){
 assert(key==0x42524745U&&*n==8);++reads;
 if(!have)return 1;
 if(writes&&readback_fail){readback_fail=0;return 2;}
 put(p,disk+(wrong_read&&writes?1:0));return 0;
}
int persistent_store_set(uint32_t key,const void*p,size_t n){assert(key==0x42524745U&&n==8);++writes;disk=get(p);have=1;return write_error?2:0;}
int main(int argc,char**argv){
 assert(argc==2);assert(board_recovery_boot_current()==0&&reads==0&&writes==0);
 if(!strcmp(argv[1],"write_uncertain"))write_error=1;
 else if(!strcmp(argv[1],"readback"))readback_fail=1;
 else if(!strcmp(argv[1],"mismatch"))wrong_read=1;
 else if(!strcmp(argv[1],"max")){disk=UINT64_MAX;have=1;assert(board_recovery_boot_generation()==0&&writes==0);return 0;}
 else if(!strcmp(argv[1],"corrupt_zero")){disk=0;have=1;assert(board_recovery_boot_generation()==0&&writes==0);return 0;}
 else assert(!strcmp(argv[1],"success"));
 if(strcmp(argv[1],"success")){
  assert(board_recovery_boot_generation()==0&&board_recovery_boot_current()==0&&disk==1&&writes==1);
  write_error=readback_fail=wrong_read=0;
  assert(board_recovery_boot_generation()==2&&board_recovery_boot_current()==2&&disk==2&&writes==2);
 }else assert(board_recovery_boot_generation()==1&&disk==1&&writes==1);
 unsigned w=writes,r=reads;uint64_t published=board_recovery_boot_current();
 assert(board_recovery_boot_generation()==published&&writes==w&&reads==r);
 puts(argv[1]);return 0;
}
'''
class RecoveryBootTests(unittest.TestCase):
    def test_verified_generation_uncertain_writes_and_unpublished_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);(p/'persistent_store.h').write_text(HEADER);(p/'main.c').write_text(MAIN);exe=p/'boot'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(p),'-I',str(ROOT/'Core/Inc'),str(ROOT/'Core/Src/board_recovery_boot.c'),str(p/'main.c'),'-o',str(exe)],check=True)
            for case in ['success','write_uncertain','readback','mismatch','max','corrupt_zero']:
                with self.subTest(case=case):subprocess.run([str(exe),case],check=True)
