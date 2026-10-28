"""Actual RF service/IRQ tests with captured canonical ACK fixtures."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

def function(source, signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]

PREAMBLE = r'''

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#define RFBOARD_LORA_SCHEDULER 0
#define RADIO_AIR_BIT_RATE_BPS 64000U
#define RADIO_AIR_SHARE_PERCENT 40U
#define RADIO_AIR_FRAME_OVERHEAD_BYTES 16U
#define RADIO_BAUD_RATE 115200U
#define RADIO_PACKET_GAP_MS 55U
#define RADIO_PACKET_BURST_MS 100U
#define RADIO_UART_FRAME_HEADER_SIZE 4U
#define RADIO_UART_FRAME_BUF_SIZE 1028U
#define RADIO_UART_FRAME_SYNC_0 0xa5
#define RADIO_UART_FRAME_SYNC_1 0x5a
#define RADIO_UART_PRIORITY_NETWORK_CONTROL 254U
#define RADIO_UART_TX_FRAMES_PER_SERVICE 1U
#define RFBOARD_RADIO_LISTEN_ONLY 0
#define GREEN_LED_GPIO_Port 0
#define GREEN_LED_Pin 0
#define HAL_OK 0U
#define HAL_BUSY 2U
typedef unsigned HAL_StatusTypeDef;
typedef struct { uint16_t len; uint8_t priority; uint8_t is_heartbeat; uint8_t control_class; uint8_t data[1028]; uint8_t uart_frames; } radio_tx_item_t;
static radio_tx_item_t g_tx_dma_item, candidate;
static unsigned clock_ms, g_tx_quiet_until_ms, g_tx_air_budget_active;
static unsigned g_tx_packet_started_ms, g_tx_packet_until_ms, g_tx_packet_active;
static unsigned g_tx_last_forced_boundary_ms;
static unsigned g_tx_dma_busy, g_tx_dma_started_at_ms, g_tx_dma_preserves_rx, g_tx_dma_recoveries;
static unsigned g_tx_drops, g_tx_startup_delays, g_tx_errors, g_tx_busy, g_tx_dma_started;
static unsigned offered, hal_status, requeues, sends;
typedef struct {void *Instance;} UART_HandleTypeDef;
static UART_HandleTypeDef uart_device={(void*)1};
static UART_HandleTypeDef *g_huart=&uart_device;
static unsigned radio_now_ms(void){return clock_ms;}
static unsigned radio_uart_tx_timeout_ms(unsigned n){return n;}
static unsigned radio_uart_tx_ready(void){return 1;}
static unsigned radio_e22_ready_for_uart(void){return 1;}
static unsigned radio_uart_requeue_frame_front(const radio_tx_item_t *item){(void)item;++requeues;return HAL_OK;}
static void radio_uart_classify_packet_drop(unsigned a,unsigned b,unsigned c){(void)a;(void)b;(void)c;}
static uint8_t g_tx_ack_only_admission;
static uint8_t radio_uart_ack_contracts(const uint8_t*,uint16_t,uint8_t,uint64_t[4]);
static unsigned radio_uart_dequeue_frame_with_budget(radio_tx_item_t *out,unsigned budget){(void)budget;if(!offered)return 0;if(g_tx_ack_only_admission){uint64_t ids[4];if(candidate.len>196||!radio_uart_ack_contracts(candidate.data,candidate.len,candidate.priority,ids))return 0;}--offered;*out=candidate;return 1;}
static void HAL_UART_AbortTransmit(void *uart){(void)uart;}
static unsigned HAL_UART_Transmit_DMA(void *uart,uint8_t *data,unsigned n){(void)uart;(void)data;(void)n;++sends;return hal_status;}
static void HAL_GPIO_TogglePin(unsigned port,unsigned pin){(void)port;(void)pin;}
#include <stdio.h>

static uint8_t g_tx_independent_streak;
static unsigned g_radio_tx_ok, g_tx_dma_complete, g_rx_restart_errors, g_rx_errors, g_rx_restart_pending;
static volatile uint8_t g_tx_completed_service_credit_ms, g_tx_pending_service_ms;
static uint64_t g_tx_closed_ack_ids[4];
static uint8_t g_tx_closed_ack_count, g_tx_closed_ack_cursor;
static unsigned radio_uart_start_rx(void){return HAL_OK;}

'''
ACK_FIXTURES = r'''
static const uint8_t und[]={165,90,51,0,12,1,1,8,247,186,236,182,183,51,183,155,3,164,134,212,212,6,20,3,68,2,2,100,66,44,90,241,216,214,133,95,67,146,158,18,187,252,167,50,46,211,58,171,32,98,131,104,4,79,26};
static const uint8_t fc[]={165,90,50,0,12,1,1,8,149,183,236,182,183,51,185,45,225,206,200,143,10,20,3,68,2,2,40,235,83,222,98,111,66,65,95,67,146,158,18,187,252,167,223,175,42,232,164,105,163,239,132,210,156,77};
static const uint8_t health_a[]={165,90,120,0,83,68,84,2,136,185,182,227,151,10,96,130,186,236,182,183,51,217,216,3,1,0,0,0,11,0,0,0,53,117,0,0,0,0,0,0,232,3,0,0,0,0,0,0,0,0,0,0,164,144,0,0,104,181,0,0,44,253,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,83,235,26,126};
static const uint8_t health_b[]={165,90,120,0,83,68,84,2,136,229,150,244,231,15,96,233,181,236,182,183,51,179,155,3,1,0,0,0,3,0,0,0,48,117,0,0,0,0,0,0,232,3,0,0,0,0,0,0,8,0,0,0,172,72,0,0,220,90,0,0,60,11,1,0,0,0,0,0,0,0,0,0,9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,80,231,199,107};
static const uint8_t baro[]={165,90,36,0,83,68,84,2,136,224,226,162,135,3,12,187,180,236,182,183,51,229,169,2,0,0,0,0,0,80,195,71,0,248,64,194,2,6,52,227};
static unsigned normal_fixture_cursor;

'''
CHECKS = r'''

static void reset(void){
 g_tx_packet_active=g_tx_air_budget_active=g_tx_dma_busy=g_tx_dma_preserves_rx=0;
 g_tx_last_forced_boundary_ms=g_tx_completed_service_credit_ms=g_tx_pending_service_ms=0;
 g_tx_closed_ack_count=g_tx_closed_ack_cursor=0;memset(g_tx_closed_ack_ids,0,sizeof(g_tx_closed_ack_ids));
 offered=0;hal_status=HAL_OK;g_tx_independent_streak=0;g_radio_tx_ok=g_tx_dma_complete=0;normal_fixture_cursor=0;
}
static void queue(const uint8_t *bytes,unsigned len,unsigned priority){memset(&candidate,0,sizeof(candidate));candidate.len=len;candidate.priority=priority;memcpy(candidate.data,bytes,len);offered=1;}
static void normal(unsigned len){memset(&candidate,0,sizeof(candidate));candidate.len=len;candidate.priority=0;const uint8_t *bytes;if(len==124){bytes=(normal_fixture_cursor++&1)?health_b:health_a;}else{assert(len==40);bytes=baro;}memcpy(candidate.data,bytes,len);offered=1;}
static void done(unsigned now){clock_ms=now;HAL_UART_TxCpltCallback(g_huart);}
static void start(unsigned now){clock_ms=now;unsigned n=radio_uart_process_tx_with_budget(2000);if(n!=1)fprintf(stderr,"start_failed now=%u quiet=%u packetuntil=%u active=%u busy=%u\n",now,g_tx_quiet_until_ms,g_tx_packet_until_ms,g_tx_packet_active,g_tx_dma_busy);assert(n==1);}
int main(void){
 /* Actual DMA completion counts controls, never ordinary-history modulo. */
 reset();normal(124);start(1000);done(1011);assert(g_tx_independent_streak==0);
 for(unsigned quota=0;quota<7;++quota){
  queue(und,sizeof(und),255);start(1200+quota*200);done(1206+quota*200);
  assert(g_tx_independent_streak==quota+1);
 }
 normal(124);start(2800);done(2811);assert(g_tx_independent_streak==0);
 queue(und,sizeof(und),255);memcpy(candidate.data+candidate.len,fc,sizeof(fc));
 candidate.len+=sizeof(fc);candidate.uart_frames=2;start(3000);done(3010);
 assert(g_tx_independent_streak==2);
 normal(124);start(3200);done(3211);assert(g_tx_independent_streak==0);

 reset();uint64_t id;queue(fc,sizeof(fc),255);assert(radio_uart_ack_contract(candidate.data,candidate.len,candidate.priority,&id)&&id==UINT64_C(17267761553259278303));
 /* Real completed124B BOARD_HEALTH frames fund44+44 capped55, never at launch. */
 normal(124);start(30085);assert(g_tx_completed_service_credit_ms==0&&g_tx_pending_service_ms==44);done(30096);assert(g_tx_completed_service_credit_ms==44);
 normal(124);start(30151);assert(g_tx_completed_service_credit_ms==44);done(30162);assert(g_tx_completed_service_credit_ms==88);
 queue(und,sizeof(und),255);start(30195);assert(g_tx_last_forced_boundary_ms==30195&&g_tx_completed_service_credit_ms==88);done(30200);
 queue(fc,sizeof(fc),255);start(30255);unsigned debt=g_tx_quiet_until_ms;assert(g_tx_last_forced_boundary_ms==30255&&g_tx_completed_service_credit_ms==33);done(30260);
 assert(g_tx_quiet_until_ms==debt);clock_ms=30281;assert(radio_uart_air_busy());clock_ms=30314;assert(radio_uart_air_busy());clock_ms=30315;assert(!radio_uart_air_busy());
 /* Renew funding through actual completions, but retry of samelogicalFCACK
  * still cannot spend it. Owner/timestamp are not the acknowledged ID. */
 normal(40);start(30315);done(30319);normal(124);start(30334);done(30345);assert(g_tx_completed_service_credit_ms==95);
 queue(fc,sizeof(fc),255);start(30378);assert(g_tx_last_forced_boundary_ms==30255&&g_tx_completed_service_credit_ms==95);done(30383);

 /* A different owner's new ACK packet still names the same logical contract. */
 queue(fc,sizeof(fc),255);size_t header=6;uint64_t field;
 radio_read_uleb128(candidate.data,candidate.len,&header,&field);radio_read_uleb128(candidate.data,candidate.len,&header,&field);radio_read_uleb128(candidate.data,candidate.len,&header,&field);
 if(candidate.data[4]&8)radio_read_uleb128(candidate.data,candidate.len,&header,&field);
 candidate.data[header]^=1;unsigned ownercrc=launchcore_crc32(candidate.data+4,candidate.len-8);for(unsigned i=0;i<4;++i)candidate.data[candidate.len-4+i]=(ownercrc>>(8*i))&255;
 assert(radio_uart_ack_contract(candidate.data,candidate.len,candidate.priority,&id)&&id==UINT64_C(17267761553259278303));clock_ms=30384;radio_uart_close_packet_batch(&candidate);assert(g_tx_last_forced_boundary_ms==30255&&g_tx_completed_service_credit_ms==95);
 /* Independent controls and failed HAL admission/lateIRQ earn no service. */
 reset();queue(und,sizeof(und),255);start(1000);done(1005);queue(fc,sizeof(fc),255);start(1060);assert(g_tx_last_forced_boundary_ms==1000&&g_tx_completed_service_credit_ms==0);done(1065);
 reset();normal(124);clock_ms=2000;hal_status=HAL_BUSY;assert(!radio_uart_process_tx_with_budget(2000));assert(g_tx_pending_service_ms==0&&g_tx_completed_service_credit_ms==0);done(2010);assert(g_tx_completed_service_credit_ms==0);
 reset();normal(124);start(3000);clock_ms=4000;offered=0;radio_uart_process_tx_with_budget(2000);assert(!g_tx_dma_busy&&g_tx_pending_service_ms==0&&g_tx_completed_service_credit_ms==0);done(4001);assert(g_tx_completed_service_credit_ms==0);
 reset();normal(124);start(5000);HAL_UART_ErrorCallback(g_huart);assert(g_tx_pending_service_ms==0);done(5012);assert(g_tx_completed_service_credit_ms==0);
 /* CRC, encrypted/compressed, SDT and incompletebounds never grant a key. */
 queue(fc,sizeof(fc),255);candidate.data[candidate.len-1]^=1;assert(!radio_uart_ack_contract(candidate.data,candidate.len,candidate.priority,&id));
 for(unsigned flag=1;flag<=16;flag*=16){queue(fc,sizeof(fc),255);candidate.data[4]|=flag;assert(!radio_uart_ack_contract(candidate.data,candidate.len,candidate.priority,&id));}
 normal(124);candidate.priority=255;assert(!radio_uart_ack_contract(candidate.data,candidate.len,candidate.priority,&id));
 queue(fc,sizeof(fc),255);candidate.len=8;assert(!radio_uart_ack_contract(candidate.data,candidate.len,candidate.priority,&id));
 queue(fc,sizeof(fc),255);candidate.len=UINT16_MAX;assert(!radio_uart_ack_contract(candidate.data,candidate.len,candidate.priority,&id));
 queue(fc,sizeof(fc),255);memset(candidate.data+6,0x80,candidate.len-6-12);unsigned crc=launchcore_crc32(candidate.data+4,candidate.len-8);for(unsigned i=0;i<4;++i)candidate.data[candidate.len-4+i]=(crc>>(8*i))&255;assert(!radio_uart_ack_contract(candidate.data,candidate.len,candidate.priority,&id));
 /* Tick wrap keeps the old elapsed/stopbit gate semantics. */
 reset();g_tx_completed_service_credit_ms=55;queue(und,sizeof(und),255);start(UINT32_MAX-70);done(UINT32_MAX-65);
 queue(fc,sizeof(fc),255);start(UINT32_MAX-10);assert(g_tx_completed_service_credit_ms==0);done(UINT32_MAX-5);clock_ms=48;assert(radio_uart_air_busy());clock_ms=49;assert(!radio_uart_air_busy());

 /* Under-capacity continuous source+uniqueACK traffic uses actual service,
  * ISR completion and gates; unchanged seven-control/FIFO turn determines
  * the offered next candidate. No future wait or extra budget is supplied. */
 reset();unsigned ai=0,ao=0,ni=0,no=0,completes=0,finish_at=0,kind=0,maxnormal=0,maxack=0;
 unsigned ack_times[200],normal_times[60];
 for(unsigned now=0;now<=10000;++now){
  if(g_tx_dma_busy&&now>=finish_at){done(now);++completes;}
  if(now<=8000&&now%50==0)ack_times[ai++]=now;
  if(now<=8000&&now%200==0)normal_times[ni++]=now;
  clock_ms=now;
  if(g_tx_dma_busy||radio_uart_air_busy())continue;
  unsigned choose_ack=ao<ai&&(g_tx_independent_streak<7U||no==ni);
  if(choose_ack){
   queue(fc,sizeof(fc),255);uint64_t key=100000U+ack_times[ao];
   for(unsigned i=0;i<8;++i)candidate.data[candidate.len-12+i]=(key>>(8*i))&255;
   unsigned crc=launchcore_crc32(candidate.data+4,candidate.len-8);for(unsigned i=0;i<4;++i)candidate.data[candidate.len-4+i]=(crc>>(8*i))&255;
   unsigned age=now-ack_times[ao];if(age>maxack)maxack=age;kind=1;
  }else if(no<ni){normal(124);unsigned age=now-normal_times[no];if(age>maxnormal)maxnormal=age;kind=2;}else continue;
  if(radio_uart_process_tx_with_budget(2000)){finish_at=now+(candidate.len*10000U+115199U)/115200U;if(kind==1)++ao;else ++no;}
 }
 fprintf(stderr,"RF_CONTINUOUS normal=%u/%u ACK=%u/%u max_normal_ms=%u max_ACK_ms=%u completed=%u\n",no,ni,ao,ai,maxnormal,maxack,completes);
 assert(no==ni&&ao==ai&&maxnormal<=1500&&maxack<=500);
 return 0;
}

'''

class FundedAckBoundaryTests(unittest.TestCase):
    def _run(self, checks):
        source = (ROOT / 'Core/Src/radio.c').read_text()
        code = function(source, 'static uint8_t radio_read_uleb128(')
        code += function(source[source.index('/* Only canonical packets are independent'):], 'static uint8_t radio_uart_independent_control(')
        code += source[source.index('static uint32_t radio_uart_air_ms(uint16_t len)\n{'):source.index('uint8_t radio_uart_tx_ready(void)')]
        code += function(source, 'uint32_t radio_uart_process_tx_with_budget(uint32_t budget_ms)\n{')
        code += function(source, 'void HAL_UART_TxCpltCallback(')
        code += function(source, 'void HAL_UART_ErrorCallback(')
        crc = function((ROOT / 'tests/radio_crc_reference.h').read_text(), 'uint32_t launchcore_crc32(')
        with tempfile.TemporaryDirectory() as directory:
            exe = str(Path(directory) / 'funded-ack')
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined', '-x', 'c', '-', '-o', exe], input=PREAMBLE + next(line for line in source.splitlines() if line.startswith("#define RADIO_PACKET_SERVICE_CREDIT_CAP_MS ")) + "\n" + crc + code + ACK_FIXTURES + checks, text=True, check=True)
            subprocess.run([exe], check=True)

    def test_actual_dma_completion_funds_only_new_plaintext_ack_boundaries(self):
        self._run(CHECKS)

    def test_completed_discovery_airtime_funds_the_same_bounded_ack_close(self):
        captured = (ROOT / 'tests/fixtures/radio-startup-discovery-frame.bin').read_bytes()
        assert len(captured) == 196 and captured[4:7] == b'SDT'
        code = 'static const uint8_t discovery[]={' + ','.join(map(str, captured)) + '};\n'
        self._run(CHECKS[:CHECKS.index('int main(void)')] + code + r'''int main(void){
 reset();queue(discovery,sizeof(discovery),254);start(1000);
 assert(g_tx_completed_service_credit_ms==0 && g_tx_pending_service_ms==67);
 done(1018);assert(g_tx_completed_service_credit_ms==67);
 queue(und,sizeof(und),255);start(1068);done(1073);
 assert(g_tx_last_forced_boundary_ms==1068 && g_tx_completed_service_credit_ms==67);
 queue(fc,sizeof(fc),255);start(1128);done(1133);
 assert(g_tx_last_forced_boundary_ms==1128 && g_tx_completed_service_credit_ms==12);
 normal(124);clock_ms=1170;assert(!radio_uart_process_tx_with_budget(2000));
 assert(offered==1 && g_tx_completed_service_credit_ms==12);
 clock_ms=1188;assert(radio_uart_process_tx_with_budget(2000)==1);
 reset();queue(discovery,sizeof(discovery),254);hal_status=HAL_BUSY;
 clock_ms=2000;assert(!radio_uart_process_tx_with_budget(2000));done(2018);
 assert(g_tx_pending_service_ms==0 && g_tx_completed_service_credit_ms==0);
 reset();queue(discovery,sizeof(discovery),254);start(3000);
 HAL_UART_ErrorCallback(g_huart);done(3018);assert(g_tx_completed_service_credit_ms==0);
 /* Coalesced receipts remain independent controls: aggregate UART length
  * must never make ACK-only traffic look like completed ordinary service. */
 reset();queue(und,sizeof(und),255);memcpy(candidate.data+candidate.len,fc,sizeof(fc));
 candidate.len+=sizeof(fc);candidate.uart_frames=2;start(4000);
 assert(g_tx_pending_service_ms==0);done(4010);assert(g_tx_completed_service_credit_ms==0);
 return 0;
}''')

    def test_one_canonical_ack_can_borrow_only_repaid_ordinary_airtime(self):
        captured = (ROOT / 'tests/fixtures/radio-startup-discovery-frame.bin').read_bytes()
        code = 'static const uint8_t discovery[]={' + ','.join(map(str, captured)) + '};\n'
        self._run(CHECKS[:CHECKS.index('int main(void)')] + code + r'''int main(void){
 reset();queue(discovery,sizeof(discovery),254);start(1000);done(1018);
 assert(g_tx_quiet_until_ms==1067);
 queue(und,sizeof(und),255);clock_ms=1020;
 assert(radio_uart_process_tx_with_budget(2000)==1);
 assert(g_tx_quiet_until_ms==1090); /* Previous67ms plus full23ms ACK cost. */
 done(1025);queue(fc,sizeof(fc),255);clock_ms=1026;
 assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
 clock_ms=1089;assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
 clock_ms=1090;assert(radio_uart_process_tx_with_budget(2000)==1);done(1095);
 /* A coalesced ACK group is still only one loan and earns no service. */
 reset();queue(discovery,sizeof(discovery),254);start(1000);done(1018);
 queue(und,sizeof(und),255);memcpy(candidate.data+candidate.len,fc,sizeof(fc));
 candidate.len+=sizeof(fc);candidate.uart_frames=2;clock_ms=1020;
 assert(radio_uart_process_tx_with_budget(2000)==1&&g_tx_quiet_until_ms==1107);
 done(1030);assert(g_tx_independent_streak==2&&g_tx_completed_service_credit_ms==67);
 queue(fc,sizeof(fc),255);clock_ms=1031;assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
 /* Ordinary bytes, invalid receipts and the owed FIFO turn cannot borrow. */
 reset();queue(discovery,sizeof(discovery),254);start(2000);done(2018);
 normal(124);clock_ms=2020;assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
 queue(und,sizeof(und),255);candidate.data[candidate.len-1]^=1;
 assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
 queue(und,sizeof(und),255);g_tx_independent_streak=7;
 assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
 /* An existing ACK's forced quiet interval remains authoritative. */
 reset();queue(und,sizeof(und),255);start(3000);done(3005);
 queue(fc,sizeof(fc),255);clock_ms=3020;assert(!radio_uart_process_tx_with_budget(2000)&&offered==1);
 /* HAL refusal charges no bytes and must preserve previously owed debt. */
 reset();queue(discovery,sizeof(discovery),254);start(4000);done(4018);
 queue(und,sizeof(und),255);hal_status=HAL_BUSY;clock_ms=4020;
 assert(!radio_uart_process_tx_with_budget(2000));assert(g_tx_quiet_until_ms==4067);
 queue(und,sizeof(und),255);hal_status=HAL_OK;clock_ms=4021;
 assert(radio_uart_process_tx_with_budget(2000)==1 && g_tx_quiet_until_ms==4090);
 return 0;
}''')
