"""Actual shared transport/wire/floor: authority, exact duplicates and failure readbacks."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
SDK=r'''#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
typedef int SedsResult;typedef int SedsRouter;
typedef struct {uint32_t ty;size_t data_size;const char *sender;size_t sender_len;const uint32_t *endpoints;size_t num_endpoints;uint64_t timestamp;const uint8_t *payload;size_t payload_len;} SedsPacketView;
#define SEDS_OK 0
#define SEDS_BAD_ARG -10
#define SEDS_PERMISSION_DENIED -11
#define SEDS_EK_UNSIGNED 0
SedsResult seds_router_resolve_hostname_address(SedsRouter*,const char*,size_t,uint32_t*);
SedsResult seds_router_get_network_time_ms(SedsRouter*,uint64_t*);
SedsResult seds_router_log_queue_typed(SedsRouter*,uint32_t,const void*,size_t,size_t,int);
'''
PERSIST=r'''#pragma once
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
#include "board_recovery_transport.h"
#include "board_recovery_wire.h"
#include "board_recovery.h"
#include "board_recovery_floor.h"
#include "persistent_store.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static uint64_t now=3000;
static uint32_t epoch=1;
static uint8_t state=5,status[40],stored[16],have;
static unsigned sent,applies,writes,inits,reads;
static int apply_result,floor_read_error;
static void put64(uint8_t*p,uint64_t n){for(unsigned i=0;i<8;i++)p[i]=(uint8_t)(n>>(i*8));}
static void put32(uint8_t*p,uint32_t n){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(n>>(i*8));}
int persistent_store_init(void){++inits;return 0;}
int persistent_store_get(uint32_t key,void*p,size_t*n){assert(key==0x4252464cU&&*n==16);++reads;if(floor_read_error){return 2;}if(!have){return 1;}memcpy(p,stored,16);return 0;}
int persistent_store_set(uint32_t key,const void*p,size_t n){assert(key==0x4252464cU&&n==16);memcpy(stored,p,n);have=1;++writes;return 0;}
uint64_t board_recovery_boot_generation(void){return 7;}
uint64_t board_recovery_boot_current(void){return 7;}
int32_t board_recovery_snapshot(BoardRecoverySnapshot*out){*out=(BoardRecoverySnapshot){.actual_state=state,.flags=BOARD_RECOVERY_SUPPORTED|BOARD_RECOVERY_STATE_VALID,.abort_epoch=epoch};return 0;}
int32_t board_recovery_apply_safe(uint8_t desired,uint32_t expected,BoardRecoverySnapshot*out){
 ++applies;
 if(apply_result){board_recovery_snapshot(out);return apply_result;}
 if(desired!=0){board_recovery_snapshot(out);return -10;}
 if(expected==epoch){state=0;++epoch;}
 else if(!(expected+1==epoch&&state==0)){board_recovery_snapshot(out);return -10;}
 return board_recovery_snapshot(out);
}
SedsResult seds_router_resolve_hostname_address(SedsRouter*r,const char*s,size_t n,uint32_t*out){assert(r&&n==2&&!memcmp(s,"GS",2));*out=1234;return 0;}
SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){assert(r);*out=now;return 0;}
SedsResult seds_router_log_queue_typed(SedsRouter*r,uint32_t ty,const void*p,size_t n,size_t size,int kind){assert(r&&ty==SEDS_DT_BOARD_STATE_STATUS&&n==40&&size==1&&kind==SEDS_EK_UNSIGNED);memcpy(status,p,40);++sent;return 0;}
int main(int argc,char**argv){
 assert(argc==2);SedsRouter router=0;uint32_t endpoint=SEDS_EP_BOARD_RECOVERY;
 uint8_t wire[32]={1,1,0,0};put64(wire+4,1);put64(wire+12,7);put32(wire+20,1);put64(wire+24,42);
 SedsPacketView packet={.ty=SEDS_DT_BOARD_RECOVERY_REQUEST,.sender="GS",.sender_len=2,.endpoints=&endpoint,.num_endpoints=1,.timestamp=3000,.payload=wire,.payload_len=32};
 board_recovery_transport_set_router(&router);
 if(!strcmp(argv[1],"unsupported_idle")){
  wire[2]=1;
  assert(board_recovery_handle(&router,&packet,endpoint,99)==0&&sent==1);
  printf("unsupportedIdle result=%u init=%u read=%u writes=%u owner=%u\n",status[1],inits,reads,writes,applies);fflush(stdout);
  assert(status[1]==BOARD_RECOVERY_RESULT_INVALID&&status[2]==5&&status[20]==1);
  assert(applies==0&&writes==0&&inits==0&&reads==0);
  assert(state==5&&epoch==1);
  /* Rejected intent was not remembered and cannot poison a valid Startup. */
  wire[2]=0;
  assert(board_recovery_handle(&router,&packet,endpoint,99)==0&&sent==2);
  assert(status[1]==BOARD_RECOVERY_RESULT_OK&&state==0&&epoch==2);
  assert(applies==1&&writes==1);puts(argv[1]);return 0;
 }
 if(!strcmp(argv[1],"io"))apply_result=-9;
 else if(!strcmp(argv[1],"pending"))apply_result=-14;
 else if(!strcmp(argv[1],"stale"))apply_result=-10;
 else if(!strcmp(argv[1],"query_fault")){wire[1]=0;floor_read_error=1;}
 else if(!strcmp(argv[1],"authority")){
  packet.sender="PB";assert(board_recovery_handle(&router,&packet,endpoint,99)==SEDS_PERMISSION_DENIED&&sent==0&&applies==0&&writes==0);
  packet.sender="@addr:1234";packet.sender_len=10;assert(board_recovery_handle(&router,&packet,endpoint,99)==0&&status[1]==0&&applies==1);
  return 0;
 }else assert(!strcmp(argv[1],"duplicate"));
 assert(board_recovery_handle(&router,&packet,endpoint,99)==0&&sent==1);
 assert(status[0]==1&&status[2]==state&&status[3]==(BOARD_RECOVERY_SUPPORTED|BOARD_RECOVERY_STATE_VALID));
 assert(status[12]==7&&status[20]==epoch&&status[24]==99);
 assert(!(status[3]&(BOARD_RECOVERY_OUTPUTS_VALID|BOARD_RECOVERY_SAFE_OUTPUTS)));
 if(!strcmp(argv[1],"io"))assert(status[1]==BOARD_RECOVERY_RESULT_IO&&state==5&&epoch==1);
 else if(!strcmp(argv[1],"pending"))assert(status[1]==BOARD_RECOVERY_RESULT_PENDING&&state==5&&epoch==1);
 else if(!strcmp(argv[1],"stale"))assert(status[1]==BOARD_RECOVERY_RESULT_STALE&&state==5&&epoch==1);
 else if(!strcmp(argv[1],"query_fault"))assert(status[1]==BOARD_RECOVERY_RESULT_IO&&state==5&&applies==0&&writes==0);
 else{
  assert(status[1]==0&&state==0&&epoch==2&&applies==1&&writes==1);
  assert(board_recovery_handle(&router,&packet,endpoint,99)==0&&status[1]==0&&epoch==2&&applies==2&&writes==1);
  wire[20]=2;assert(board_recovery_handle(&router,&packet,endpoint,99)==0&&status[1]==BOARD_RECOVERY_RESULT_INVALID&&applies==2&&writes==1);
 }
 puts(argv[1]);return 0;
}
'''
class RecoveryTransportTests(unittest.TestCase):
    def test_actual_shared_transport_floor_and_result_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);(p/'sedsnet.h').write_text(SDK);(p/'persistent_store.h').write_text(PERSIST);(p/'main.c').write_text(MAIN);exe=p/'transport'
            sources=[ROOT/'Core/Src'/n for n in ['board_recovery_transport.c','board_recovery_wire.c','board_recovery_floor.c']]
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(p),'-I',str(ROOT/'Core/Inc'),*[str(s)for s in sources],str(p/'main.c'),'-o',str(exe)],check=True)
            for case in ['io','pending','stale','query_fault','authority','duplicate','unsupported_idle']:
                with self.subTest(case=case):subprocess.run([str(exe),case],check=True)
