"""Exercise actual RF DMA service and packet gates without physical outputs."""
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


class CriticalPacketBoundaryTests(unittest.TestCase):
    def test_accepted_critical_dma_closes_only_one_batch_per_normal_cycle(self):
        source = (ROOT / 'Core/Src/radio.c').read_text()
        gates = source[source.index('static uint32_t radio_uart_air_ms(uint16_t len)\n{'):
                       source.index('uint8_t radio_uart_tx_ready(void)')]
        read = function(source, 'static uint8_t radio_read_uleb128(')
        # The first occurrence is a declaration; select the actual definition.
        start = source.index('/* Only canonical packets are independent')
        independent = function(source[start:], 'static uint8_t radio_uart_independent_control(')
        service = function(source, 'uint32_t radio_uart_process_tx_with_budget(uint32_t budget_ms)\n{')
        preamble = r'''
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
static void *g_huart=(void *)1;
static unsigned radio_now_ms(void){return clock_ms;}
static unsigned radio_uart_tx_timeout_ms(unsigned n){return n;}
static unsigned radio_uart_tx_ready(void){return 1;}
static unsigned radio_e22_ready_for_uart(void){return 1;}
static unsigned radio_uart_requeue_frame_front(const radio_tx_item_t *item){(void)item;++requeues;return HAL_OK;}
static void radio_uart_classify_packet_drop(unsigned a,unsigned b,unsigned c){(void)a;(void)b;(void)c;}
static unsigned radio_uart_dequeue_frame_with_budget(radio_tx_item_t *out,unsigned budget){(void)budget;if(!offered)return 0;--offered;*out=candidate;return 1;}
static void HAL_UART_AbortTransmit(void *uart){(void)uart;}
static unsigned HAL_UART_Transmit_DMA(void *uart,uint8_t *data,unsigned n){(void)uart;(void)data;(void)n;++sends;return hal_status;}
static void HAL_GPIO_TogglePin(unsigned port,unsigned pin){(void)port;(void)pin;}
'''
        preamble += """
static volatile uint8_t g_tx_completed_service_credit_ms, g_tx_pending_service_ms;
static uint64_t g_tx_closed_ack_ids[4];
static uint8_t g_tx_closed_ack_count, g_tx_closed_ack_cursor;
"""
        preamble += function((ROOT / 'tests/radio_crc_reference.h').read_text(), 'uint32_t launchcore_crc32(')
        preamble += "\n" + next(line for line in source.splitlines() if line.startswith("#define RADIO_PACKET_SERVICE_CREDIT_CAP_MS ")) + "\n"
        test = r'''
static void setup(unsigned priority, int sdt, unsigned len){
 memset(&candidate,0,sizeof(candidate));candidate.len=len;candidate.priority=priority;
 candidate.data[0]=0xa5;candidate.data[1]=0x5a;
 candidate.data[2]=(len-4)&255;candidate.data[3]=(len-4)>>8;
 if(sdt)memcpy(candidate.data+4,"SDT",3);else candidate.data[6]=1;
 offered=1;hal_status=HAL_OK;
}
static void reset(void){g_tx_packet_active=0;g_tx_air_budget_active=0;g_tx_dma_busy=0;g_tx_last_forced_boundary_ms=0;}
static void drained(unsigned at){clock_ms=at;g_tx_dma_busy=0;radio_uart_mark_tx_drained();}
int main(void){
 /* Captured first ACK starts UART at9256.959ms and was followed by92ms
  * of ordinary bytes. Closing here must preserve the actual stop-bit gap. */
 reset();clock_ms=9256;setup(255,0,55);
 assert(radio_uart_independent_control(candidate.data,candidate.len,candidate.priority));
 assert(radio_uart_process_tx_with_budget(2000)==1);
 assert(g_tx_quiet_until_ms==9279); /* Airtime charge is unchanged. */
 drained(9262);clock_ms=9280;assert(radio_uart_air_busy());
 clock_ms=9316;assert(radio_uart_air_busy());
 clock_ms=9317;assert(!radio_uart_air_busy());
 assert(g_tx_last_forced_boundary_ms==9256);
 /* Repeated ACKs cannot force every frame into its own modem packet. The
  * normal burst admits ordinary work while the155ms promotion limit runs. */
 unsigned ordinary=0;
 for(unsigned at=9317;at<=9393;at+=19){
  clock_ms=at;assert(!radio_uart_air_busy());setup(at==9355?16:255,0,42);
  assert(radio_uart_process_tx_with_budget(2000)==1);
  if(at==9355)++ordinary;
  drained(at+4);assert(g_tx_last_forced_boundary_ms==9256);
 }
 assert(ordinary==1);
 clock_ms=9412;assert(!radio_uart_air_busy());setup(254,0,42);
 assert(radio_uart_process_tx_with_budget(2000)==1);drained(9416);
 assert(g_tx_last_forced_boundary_ms==9412);
 clock_ms=9431;assert(radio_uart_air_busy());clock_ms=9471;assert(!radio_uart_air_busy());
 /* A forged high-priority SDT remains dependent and cannot close a batch. */
 reset();clock_ms=1000;setup(255,1,42);assert(radio_uart_process_tx_with_budget(2000)==1);
 drained(1004);clock_ms=1019;assert(!radio_uart_air_busy());assert(g_tx_last_forced_boundary_ms==0);
 /* Refused DMA never spends the bounded close opportunity. */
 reset();clock_ms=2000;setup(255,0,42);hal_status=HAL_BUSY;
 assert(!radio_uart_process_tx_with_budget(2000));assert(requeues==1);
 assert(g_tx_last_forced_boundary_ms==0 && g_tx_packet_active==0);
 /* Delayed completion and tick wrap keep quiet anchored to actual drain. */
 reset();clock_ms=UINT32_MAX-20U;setup(255,0,42);assert(radio_uart_process_tx_with_budget(2000)==1);
 drained(UINT32_MAX-15U);clock_ms=38;assert(radio_uart_air_busy());clock_ms=39;assert(!radio_uart_air_busy());
 clock_ms=40;setup(255,0,42);assert(radio_uart_process_tx_with_budget(2000)==1);
 assert(g_tx_last_forced_boundary_ms==UINT32_MAX-20U);
 /* Closing never releases a longer existing airtime reservation. */
 reset();clock_ms=3000;setup(255,0,1028);assert(radio_uart_process_tx_with_budget(2000)==1);
 drained(3090);clock_ms=3326;assert(radio_uart_air_busy());clock_ms=3327;assert(!radio_uart_air_busy());
}
'''
        with tempfile.TemporaryDirectory() as directory:
            exe = str(Path(directory) / 'critical-boundary')
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',exe],
                           input=preamble+read+independent+gates+service+test,text=True,check=True)
            subprocess.run([exe],check=True)
