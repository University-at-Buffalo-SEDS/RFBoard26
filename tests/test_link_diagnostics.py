import unittest, subprocess, tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
class LinkDiagnosticsTests(unittest.TestCase):
 def test_error_capture_and_bounded_report_use_production_code(self):
  source=(ROOT/'Core/Src/telemetry.c').read_text()
  code=source[source.index('/* Read-only diagnostic counters'):source.index('static void telemetry_can_rx(')]
  fields=['rx_frames_ok','rx_isr_drops','rx_bad_len','rx_errors','rx_restart_errors','tx_ok','tx_errors','tx_busy','tx_drops','tx_queue_count','tx_dma_recoveries','rx_isr_bytes','rx_sync_loss']
  stub="""
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#define SEDS_OK 0
#define SEDS_EK_UNSIGNED 1
typedef int SedsResult;typedef unsigned SedsDataType;
static struct {void *r;} g_router={(void*)1};
static uint32_t now, logs, captured[32],g_pending_can_count,g_pending_can_drops,g_telemetry_discovery_seen,g_radio_link_seen;
volatile uint32_t g_watchdog_reset_flags,g_watchdog_feed_count,g_telemetry_loop_completions;
volatile uint32_t g_av_bay_underglow_updates,g_av_bay_underglow_persist_errors,g_av_bay_underglow_enabled,g_av_bay_underglow_persist_writes;
static int rx_result;
static uint32_t HAL_GetTick(void){return now;}
static int seds_router_receive_packed_from_side(void*r,uint32_t side,const uint8_t*d,size_t n){assert(r&&side==2&&d&&n);return rx_result;}
static int seds_router_receive_packed(void*r,const uint8_t*d,size_t n){assert(r&&d&&n);return rx_result;}
"""+'typedef struct {'+''.join('uint32_t '+f+';' for f in fields)+'} radio_uart_stats_t;\n'+"""
static radio_uart_stats_t radio_uart_stats_snapshot(void){radio_uart_stats_t r={0};r.rx_isr_drops=7;return r;}
static int seds_router_log_typed(void*r,unsigned ty,const void*d,size_t n,size_t size,int kind){assert(r&&ty==1000&&n==32&&size==4&&kind==1);memcpy(captured,d,128);logs++;return -14;}
"""
  main="""
int main(void){uint8_t data=1;
 rx_result=-13;assert(telemetry_observe_receive(&data,1,2,&g_rf_radio_rx_fail,&g_rf_radio_rx_last)==-13);
 assert(g_rf_radio_rx_fail==1&&g_rf_radio_rx_last==-13);
 rx_result=0;assert(telemetry_observe_receive(&data,1,-1,&g_rf_radio_rx_fail,&g_rf_radio_rx_last)==0);
 assert(g_rf_radio_rx_fail==1&&g_rf_radio_rx_last==0);
 now=4999;telemetry_publish_link_diagnostics();assert(logs==0);
 g_watchdog_reset_flags=0x20000000;now=5000;telemetry_publish_link_diagnostics();
 assert(logs==1&&captured[0]==1&&captured[1]==5000&&captured[2]==0x20000000&&captured[5]==1&&captured[12]==7);
 now=5001;telemetry_publish_link_diagnostics();assert(logs==1);
 now=10000;telemetry_publish_link_diagnostics();assert(logs==2);
}
"""
  with tempfile.TemporaryDirectory() as tmp:
   exe=Path(tmp)/'diagnostic'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=stub+code+main,text=True,check=True)
   subprocess.run([str(exe)],check=True)
