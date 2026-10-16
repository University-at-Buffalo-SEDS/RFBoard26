"""Compile the actual RF owned-TX queue under sanitizers; no hardware outputs."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class RadioControlFifoTests(unittest.TestCase):
    def test_ack_bypasses_schema_without_losing_template_order_or_bounding(self):
        source = (ROOT/'Core/Src/radio.c').read_text()
        types = '\n'.join(re.search(r'typedef struct \{[^}]*\} '+name+';', source).group()
                          for name in ['radio_tx_item_t', 'radio_tx_queue_item_t'])
        read = source[source.index('static uint8_t radio_read_uleb128('):source.index('static uint32_t radio_uart_flow_id_from_payload(')]
        remove = source[source.index('static void radio_uart_remove_queue_index('):source.index('static void radio_uart_drop_stale_locked(')]
        start = source.index('static uint8_t radio_uart_independent_control(', source.index('static HAL_StatusTypeDef radio_uart_enqueue_sedsnet_frame('))
        independent = source[start:source.index('/* Put an item removed for transmission', start)]
        start = source.index('static HAL_StatusTypeDef radio_uart_enqueue_sedsnet_frame(')
        enqueue = source[start:source.index('/* Only canonical packets', start)]
        preamble = r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
typedef unsigned HAL_StatusTypeDef;
#define HAL_OK 0U
#define HAL_ERROR 1U
#define HAL_BUSY 2U
#define RADIO_UART_FRAME_HEADER_SIZE 4U
#define RADIO_UART_FRAME_BUF_SIZE 1028U
#define RADIO_UART_TX_QUEUE_SLOTS 12U
#define RADIO_UART_TX_QUEUE_BYTE_CAPACITY 2056U
#define RADIO_UART_TX_CONTROL_RESERVE_BYTES 256U
#define RADIO_UART_TX_CONTROL_RESERVE_SLOTS 2U
#define RADIO_UART_FRAME_SYNC_0 0xa5
#define RADIO_UART_FRAME_SYNC_1 0x5a
#define RADIO_UART_PRIORITY_NETWORK_CONTROL 254U
#define RADIO_UART_SCHED_FALLBACK_FLOW 0U
#define RADIO_UART_CLASS_NETWORK_CONTROL 2U
#define RADIO_UART_CLASS_APPLICATION 0U
'''
        globals_ = r'''
static radio_tx_queue_item_t g_tx_queue[RADIO_UART_TX_QUEUE_SLOTS];
static uint8_t g_tx_queue_bytes[RADIO_UART_TX_QUEUE_BYTE_CAPACITY];
static uint32_t g_tx_count, g_tx_queue_bytes_used, g_tx_busy, g_tx_enqueued, g_tx_last_flow_id, g_tx_budget_misses, g_radio_tx_ok;
static unsigned radio_uart_lock_tx_queue(void){return HAL_OK;}
static void radio_uart_unlock_tx_queue(void){}
static uint32_t radio_now_ms(void){return 1;}
static uint32_t radio_uart_air_ms(uint16_t n){return n;}
'''
        test = r'''
static void frame(uint8_t *p,unsigned n,unsigned marker,int sdt) {
 memset(p,0,n);p[0]=0xa5;p[1]=0x5a;p[2]=(n-4)&255;p[3]=(n-4)>>8;
 if(sdt){memcpy(p+4,"SDT",3);p[7]=marker;}
 else {p[4]=0xca;p[5]=1;p[6]=1;p[7]=marker;}
}
static void reset(void){g_tx_count=0;g_tx_queue_bytes_used=0;g_radio_tx_ok=0;}
int main(void) {
 uint8_t schema[1028],ack[20],small[16];radio_tx_item_t out;
 frame(schema,sizeof(schema),10,1);frame(ack,sizeof(ack),0,0);
 assert(!radio_uart_independent_control(schema,sizeof(schema),255));
 assert(radio_uart_independent_control(ack,sizeof(ack),255));
 assert(!radio_uart_independent_control(ack,sizeof(ack),16));
 assert(radio_uart_enqueue_sedsnet_frame(schema,sizeof(schema),254)==HAL_OK);
 assert(radio_uart_enqueue_sedsnet_frame(schema,sizeof(schema),254)==HAL_BUSY);
 for(unsigned i=1;i<=3;i++){ack[7]=i;assert(radio_uart_enqueue_sedsnet_frame(ack,sizeof(ack),255)==HAL_OK);}
 assert(g_tx_count==4);
 for(unsigned i=1;i<=3;i++){
  assert(radio_uart_dequeue_frame_with_budget(&out,2000));assert(out.data[7]==i);
  g_radio_tx_ok++;
 }
 assert(radio_uart_dequeue_frame_with_budget(&out,2000)&&out.data[7]==10);
 assert(!g_tx_count&&!g_tx_queue_bytes_used);
 reset();
 for(unsigned i=0;i<2;i++){frame(small,sizeof(small),10+i,1);assert(radio_uart_enqueue_sedsnet_frame(small,sizeof(small),254)==HAL_OK);}
 assert(radio_uart_enqueue_sedsnet_frame(ack,sizeof(ack),255)==HAL_OK);
 assert(radio_uart_dequeue_frame_with_budget(&out,2000)&&out.data[7]==3);g_radio_tx_ok++;
 for(unsigned i=0;i<2;i++){assert(radio_uart_dequeue_frame_with_budget(&out,2000));assert(out.data[7]==10+i);g_radio_tx_ok++;}
 reset();frame(small,sizeof(small),0,1);
 for(unsigned i=0;i<10;i++)assert(radio_uart_enqueue_sedsnet_frame(small,sizeof(small),254)==HAL_OK);
 assert(radio_uart_enqueue_sedsnet_frame(small,sizeof(small),254)==HAL_BUSY);
 for(unsigned i=0;i<2;i++)assert(radio_uart_enqueue_sedsnet_frame(ack,sizeof(ack),255)==HAL_OK);
 assert(radio_uart_enqueue_sedsnet_frame(ack,sizeof(ack),255)==HAL_BUSY);
 g_radio_tx_ok=7;
 assert(radio_uart_dequeue_frame_with_budget(&out,2000)&&memcmp(out.data+4,"SDT",3)==0);
 g_radio_tx_ok=8;
 assert(radio_uart_dequeue_frame_with_budget(&out,2000)&&out.data[6]==1);
 assert(g_tx_count==10);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            exe=str(Path(tmp)/'queue-test')
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',exe],
                           input=preamble+types+globals_+read+remove+independent+enqueue+test,text=True,check=True)
            subprocess.run([exe],check=True)
