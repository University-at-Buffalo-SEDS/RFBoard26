from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
class CanOwnershipTests(unittest.TestCase):
    def test_full_pending_queue_retains_accepted_frames_and_fifo_order(self):
        s=(ROOT/'Core/Src/telemetry.c').read_text()
        identifier=s[s.index('static uint32_t telemetry_flight_can_id('):s.index('#define TELEMETRY_PENDING_CAN_DEPTH')]
        funcs=s[s.index('static bool telemetry_enqueue_pending_can_command'):s.index('static bool telemetry_unix_ms_to_utc')]
        code=r'''
#include <assert.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define TELEMETRY_PENDING_CAN_DEPTH 3U
#define TELEMETRY_PENDING_CAN_MAX_LEN 128U
typedef int SedsResult;
typedef int HAL_StatusTypeDef;
enum {SEDS_OK,SEDS_IO,HAL_OK=0,HAL_BUSY=1};
typedef struct {size_t len;uint32_t can_id;uint8_t data[128];} TelemetryPendingCanCommand;
static TelemetryPendingCanCommand g_pending_can[3];
static unsigned g_pending_can_count,g_pending_can_head,g_pending_can_tail,g_pending_can_drops;
static unsigned busy=1,calls,sent;static uint8_t seen[8];
#define SEDS_DT_HEARTBEAT 120
#define TELEMETRY_FLIGHT_CAN_ID 0x101U
#define TELEMETRY_FLIGHT_HEARTBEAT_CAN_ID 0x001U
static uint32_t sim_probe_packed_data_type(const uint8_t *p,size_t n){(void)p;(void)n;return 0;}
''' + identifier + r'''
static int can_bus_send_large(const uint8_t *p,size_t n,uint32_t id){assert(n==1 && id==(*p==1?0x101:0x001));calls++;if(busy)return HAL_BUSY;seen[sent++]=*p;return HAL_OK;}
''' + funcs+r'''
int main(void){
    uint8_t a=1,b=2,c=3,d=4;
    uint8_t opaque[3][8]={{83,68,84,1},{83,68,84,2},{83,68,7,1}};
    for(unsigned i=0;i<3;i++) for(unsigned p=0;p<256;p++)
      assert(telemetry_flight_can_id(opaque[i],8,p)==(p>=200?0x001:0x101));
    assert(telemetry_send_or_queue_can_packet(&a,1,0)==SEDS_OK);
    assert(telemetry_send_or_queue_can_packet(&b,1,255)==SEDS_OK);
    assert(telemetry_send_or_queue_can_packet(&c,1,200)==SEDS_OK);
    assert(calls==1); /* only the first may attempt hardware while backlog exists */
    assert(telemetry_send_or_queue_can_packet(&d,1,254)==SEDS_IO);
    assert(g_pending_can_count==3 && g_pending_can_drops==1);
    telemetry_retry_pending_can_commands();assert(g_pending_can_count==3 && sent==0);
    busy=0;telemetry_retry_pending_can_commands();
    assert(g_pending_can_count==0 && sent==3);
    assert(seen[0]==1 && seen[1]==2 && seen[2]==3);
    assert(telemetry_send_or_queue_can_packet(&d,1,254)==SEDS_OK);
    assert(sent==4 && seen[3]==4);
}
'''
        self.compile_run(code)
    def test_full_hardware_fifo_does_not_abort_accepted_frames(self):
        s=(ROOT/'Core/Src/can_bus.c').read_text()
        fn=s[s.index('static HAL_StatusTypeDef can_bus_enqueue_tx_frame'):s.index('static inline void can_bus_notify_rx')]
        code=r'''
#include <stdint.h>
#include <assert.h>
typedef int HAL_StatusTypeDef;
typedef int FDCAN_TxHeaderTypeDef;
enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT};
#define CAN_BUS_TX_ENQUEUE_TIMEOUT_MS 5
#define FDCAN_TX_BUFFER0 1
#define FDCAN_TX_BUFFER1 2
#define FDCAN_TX_BUFFER2 4
static int handle,*g_hfdcan=&handle;
static unsigned tick,free_slots,aborts,accepted,g_fdcan_tx_fail_count,g_fdcan_tx_ok_count;
static unsigned HAL_GetTick(void){return tick++;}
static unsigned HAL_FDCAN_GetTxFifoFreeLevel(int *h){(void)h;return free_slots;}
static int can_bus_recover_if_bus_off(void){return HAL_OK;}
static int HAL_FDCAN_AbortTxRequest(int *h,unsigned bits){(void)h;(void)bits;aborts++;return HAL_OK;}
static int HAL_FDCAN_AddMessageToTxFifoQ(int *h,const int *hdr,const uint8_t *d){(void)h;(void)hdr;(void)d;accepted++;return HAL_OK;}
''' + fn + r'''
int main(void){
    (void)HAL_FDCAN_AbortTxRequest;int hdr=0;uint8_t bytes[64]={0};
    assert(can_bus_enqueue_tx_frame(&hdr,bytes)==HAL_BUSY);
    assert(aborts==0 && accepted==0 && g_fdcan_tx_fail_count==1);
    free_slots=1;assert(can_bus_enqueue_tx_frame(&hdr,bytes)==HAL_OK);
    assert(accepted==1 && g_fdcan_tx_ok_count==1);
}
'''
        self.compile_run(code)
    def compile_run(self,code):
        with tempfile.TemporaryDirectory() as d:
            exe=Path(d)/'can-ownership'
            p=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=code,text=True,capture_output=True)
            self.assertEqual(p.returncode,0,p.stderr)
            subprocess.run([str(exe)],check=True)
