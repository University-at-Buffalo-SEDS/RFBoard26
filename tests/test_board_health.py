from pathlib import Path
import json,re,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class BoardHealthTests(unittest.TestCase):
 def test_actual_producer_is_bounded_exact_and_optional(self):
  source=(ROOT/'Core/Src/board_health.c').read_text()
  source='\n'.join(line for line in source.splitlines() if not line.startswith('#include'))
  harness=r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
typedef int SedsRouter;
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_DT_BOARD_HEALTH 42
#define SEDS_EK_UNSIGNED 0
#define BOARD_UPTIME_FREQUENCY_HZ 1000U
#define SD_AVAILABLE
uint64_t uptime;
unsigned calls,lock_depth,mask,refuse;
uint32_t captured[24];
volatile uint32_t g_watchdog_reset_flags=123,g_watchdog_missing_mask=5;
volatile uint32_t g_telemetry_tlsf_live_bytes=100,g_telemetry_tlsf_peak_bytes=200,g_telemetry_tlsf_region_bytes=4096,g_telemetry_tlsf_failures=7;
volatile uint32_t g_can_tx_rejected=11,g_can_tx_high_water=12,g_gateway_uart_rx_ring_drops=13;
volatile uint32_t g_sd_raw_records_enqueued_count=0xf00d1234U,g_sd_raw_records_written_count=15,g_sd_raw_records_dropped_count=16,g_sd_raw_records_flushed_count=17,g_sd_raw_records_io_lost_count=18,g_sd_write_error_count=19,g_sd_max_write_ms=20;
volatile uint32_t g_sd_enqueued_records=0xf00d1234U,g_sd_written_records=15,g_sd_flushed_records=17,g_sd_format_errors=16,g_sd_lock_contention_drops=18,g_sd_unavailable_drops=19,g_sd_shutdown_lost_records=20,g_sd_write_errors=21;
uint64_t board_uptime_ticks(void){return uptime;}
uint32_t __get_PRIMASK(void){return mask;}
void __disable_irq(void){mask=1;}
void __set_PRIMASK(uint32_t value){mask=value;}
uint32_t board_health_can_rx_drops(void){return 31;}
uint32_t board_health_uart_rx_drops(void){return 32;}
uint32_t board_health_adc_overruns(void){return 33;}
size_t seds_memory_admission_rejected(void){return 34;}
void telemetry_lock(void){assert(!mask);++lock_depth;}
void telemetry_unlock(void){assert(lock_depth==1);--lock_depth;}
SedsResult seds_router_log_bytes_ex(SedsRouter *router,int dtype,const uint8_t *data,size_t length,const uint64_t *stamp,bool queue){
 assert(router&&dtype==42&&length==96&&!stamp&&queue);
 assert(!mask&&lock_depth==1);++calls;memcpy(captured,data,sizeof(captured));return refuse?1:0;
}
''' +source+r'''
int main(void){
 SedsRouter router=1;
 board_health_poll(NULL);assert(!calls);
#ifdef TELEMETRY_ENABLED
 uptime=100;board_health_poll(&router);assert(!calls);
 uptime=4999;board_health_poll(&router);assert(!calls);
 uptime=5000;refuse=1;board_health_poll(&router);assert(calls==1);
 for(unsigned i=0;i<1000;i++)board_health_poll(&router);
 assert(calls==1&&!lock_depth&&!mask); /* no pressure retry storm */
 uptime=10000;refuse=0;board_health_poll(&router);
 assert(calls==2&&captured[23]==1&&captured[0]==1&&captured[4]==1000);
 assert(captured[5]==123&&captured[6]==5&&captured[11]==34&&captured[12]==31);
#ifdef TELEMETRY_USE_TLSF
 assert((captured[1]&1)&&captured[7]==100&&captured[8]==200&&captured[9]==4096&&captured[10]==7);
#else
 assert(!(captured[1]&1)&&!captured[7]&&!captured[8]&&!captured[9]&&!captured[10]);
#endif
 if(captured[1]&16)assert(captured[16]==0xf00d1234U); /* no float rounding */
 uptime=(UINT64_C(1)<<42)+20000;board_health_poll(&router);
 assert(calls==3&&captured[2]==20000&&captured[3]==1024);
 uptime=(UINT64_C(1)<<42)+UINT32_MAX-2U;board_health_poll(&router);assert(calls==4);
 uptime+=4999;board_health_poll(&router);assert(calls==4);
 ++uptime;board_health_poll(&router);assert(calls==5); /* 32-bit schedule wrap */
#else
 uptime=999999;board_health_poll(&router);assert(!calls);
#endif
 return 0;
}
'''
  with tempfile.TemporaryDirectory() as tmp:
   for label,flags in [('tlsf',['-DTELEMETRY_ENABLED','-DTELEMETRY_USE_TLSF']),('threadx',['-DTELEMETRY_ENABLED']),('disabled',[])]:
    binary=Path(tmp)/label
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',*flags,'-x','c','-','-o',str(binary)],input=harness,text=True,check=True)
    subprocess.run([str(binary)],check=True)
 def test_schema_and_header_use_matching_canonical_u32_identity(self):
  definitions=json.loads((ROOT/'config/sedsnet.json').read_text())
  report=next(t for t in definitions['types'] if t['name']=='BOARD_HEALTH')
  self.assertEqual(report['element'],dict(kind='Static',data_type='UInt32',count=24))
  self.assertEqual((report['class'],report['reliable'],report['priority']),('Data','Off',0))
  hashed=0x811c9dc5 ^ 0x5459
  for byte in b'BOARD_HEALTH':hashed=((hashed ^ byte)*0x01000193)&0xffffffff
  expected=0x10000 | (hashed & 0x0fffffff)
  header=(ROOT/'Core/Inc/sedsnet_config.h').read_text()
  actual=int(re.search(r'#define SEDS_DT_BOARD_HEALTH \(\(SedsDataType\)(\d+)U\)',header)[1])
  self.assertEqual(actual,expected)
