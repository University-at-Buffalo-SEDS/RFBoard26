"""Actual floor owner: uncertain writes burn source time without weakening verified floors."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
HEADER=r'''#pragma once
#include <stddef.h>
#include <stdint.h>
typedef int launchcore_persist_status_t;
#define LAUNCHCORE_PERSIST_OK 0
#define LAUNCHCORE_PERSIST_NOT_FOUND 1
#define LAUNCHCORE_PERSIST_STORAGE_ERROR 2
int persistent_store_init(void);
int persistent_store_get(uint32_t,void*,size_t*);
int persistent_store_set(uint32_t,const void*,size_t);
'''
HARNESS=r'''
#include "board_recovery_floor.h"
#include "persistent_store.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static uint8_t disk[16];
static unsigned writes;
static int write_error,read_error,wrong_read,copy_before_error;
static void put(uint8_t *p,uint64_t n){for(unsigned i=0;i<8;i++)p[i]=(uint8_t)(n>>(i*8));}
int persistent_store_init(void){return 0;}
int persistent_store_get(uint32_t key,void *value,size_t *n){
 assert(key==0x4252464cU && *n==16);
 if(read_error)return 2;
 memcpy(value,disk,16);
 if(wrong_read)((uint8_t*)value)[0]^=0x80U;
 return 0;
}
int persistent_store_set(uint32_t key,const void *value,size_t n){
 assert(key==0x4252464cU && n==16);++writes;
 if(!write_error||copy_before_error)memcpy(disk,value,n);
 return write_error?2:0;
}
int main(int argc,char **argv){
 assert(argc==2);put(disk,1);put(disk+8,2000);
 assert(board_recovery_floor_is_healthy());assert(board_recovery_floor_active());
 assert(!board_recovery_floor_allows_recovery_time(1,2000));
 assert(board_recovery_floor_allows_recovery_time(1,2001));
 if(!strcmp(argv[1],"lower_new_issuer")){
  unsigned prior=writes;
  assert(!board_recovery_floor_allows_recovery_time(2,1000));
  assert(!board_recovery_floor_commit(2,1000)&&writes==prior);
  assert(board_recovery_floor_commit(2,2001));puts(argv[1]);return 0;
 }
 if(!strcmp(argv[1],"interleaved_issuers")){
  write_error=1;assert(!board_recovery_floor_commit(1,3000));
  assert(!board_recovery_floor_commit(2,4000));
  unsigned prior=writes;
  assert(!board_recovery_floor_allows_recovery_time(1,2500));
  assert(!board_recovery_floor_commit(1,2500)&&writes==prior);
  assert(!board_recovery_floor_commit(2,4000)&&writes==prior);
  assert(!board_recovery_command_time_allowed(99999));
  write_error=0;assert(board_recovery_floor_commit(3,5000));
  assert(!board_recovery_command_time_allowed(5000));
  assert(board_recovery_command_time_allowed(5001));puts(argv[1]);return 0;
 }
 if(!strcmp(argv[1],"write_before"))write_error=1;
 else if(!strcmp(argv[1],"write_after")){write_error=1;copy_before_error=1;}
 else if(!strcmp(argv[1],"readback"))read_error=1;
 else if(!strcmp(argv[1],"mismatch"))wrong_read=1;
 else if(!strcmp(argv[1],"new_issuer")){
  write_error=1;
  assert(!board_recovery_floor_commit(2,3000));
  assert(!board_recovery_command_time_allowed(9999));
  assert(!board_recovery_floor_allows_recovery_time(1,1999));
  assert(!board_recovery_floor_allows_recovery_time(1,2000));
  assert(!board_recovery_floor_allows_recovery_time(2,2999));
  assert(!board_recovery_floor_allows_recovery_time(2,3000));
  assert(board_recovery_floor_allows_recovery_time(2,3001));
  assert(board_recovery_floor_allows_recovery_time(3,4000));
  write_error=0;assert(board_recovery_floor_commit(3,4000));
  assert(!board_recovery_command_time_allowed(4000));assert(board_recovery_command_time_allowed(4001));
  puts(argv[1]);return 0;
 }else assert(!"unknown case");
 assert(!board_recovery_floor_commit(1,3000));
 assert(!board_recovery_floor_is_healthy());
 assert(!board_recovery_command_time_allowed(9999));
 assert(!board_recovery_floor_allows_recovery_time(1,2500));
 assert(!board_recovery_floor_allows_recovery_time(1,3000));
 assert(board_recovery_floor_allows_recovery_time(1,3001));
 unsigned prior=writes;
 assert(!board_recovery_floor_commit(1,2500)&&writes==prior);
 assert(!board_recovery_floor_commit(1,3000)&&writes==prior);
 assert(!board_recovery_floor_commit(0,4000)&&writes==prior);
 assert(!board_recovery_floor_commit(1,0)&&writes==prior);
 write_error=read_error=wrong_read=0;
 assert(board_recovery_floor_commit(1,3001));
 assert(board_recovery_floor_is_healthy()&&board_recovery_floor_active());
 assert(!board_recovery_command_time_allowed(3001));assert(board_recovery_command_time_allowed(3002));
 puts(argv[1]);return 0;
}
'''
class AttemptFloorTests(unittest.TestCase):
    def test_actual_uncertain_write_highwater_and_explicit_repair(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);(p/'persistent_store.h').write_text(HEADER);(p/'main.c').write_text(HARNESS);exe=p/'floor'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(p),'-I',str(ROOT/'Core/Inc'),str(ROOT/'Core/Src/board_recovery_floor.c'),str(p/'main.c'),'-o',str(exe)],check=True)
            for case in ['write_before','write_after','readback','mismatch','new_issuer','lower_new_issuer','interleaved_issuers']:
                with self.subTest(case=case):subprocess.run([str(exe),case],check=True)
