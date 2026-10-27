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
        stale_start = source.index('static void radio_uart_drop_stale_locked(')
        stale = source[stale_start:source.index('static HAL_StatusTypeDef radio_uart_enqueue_frame(', stale_start)]
        count_start = source.index('uint32_t radio_uart_tx_queue_count(void)')
        count = source[count_start:source.index('radio_uart_stats_t radio_uart_stats_snapshot', count_start)]
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
#define RADIO_UART_TX_MAX_AGE_MS 1500U
'''
        globals_ = r'''
static radio_tx_queue_item_t g_tx_queue[RADIO_UART_TX_QUEUE_SLOTS];
static uint8_t g_tx_queue_bytes[RADIO_UART_TX_QUEUE_BYTE_CAPACITY];
static uint32_t g_tx_count, g_tx_queue_bytes_used, g_tx_busy, g_tx_enqueued, g_tx_last_flow_id, g_tx_budget_misses, g_radio_tx_ok;
static unsigned radio_uart_lock_tx_queue(void){return HAL_OK;}
static void radio_uart_unlock_tx_queue(void){}
static uint32_t clock_ms=1, g_tx_drops, g_tx_drop_stale;
static uint32_t radio_now_ms(void){return clock_ms;}
static uint32_t radio_uart_queue_index_at_offset(uint32_t offset){return offset;}
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
 reset();
 frame(small,sizeof(small),60,1);
 assert(radio_uart_enqueue_sedsnet_frame(small,sizeof(small),254)==HAL_OK);
 frame(ack,sizeof(ack),61,0);
 assert(radio_uart_enqueue_sedsnet_frame(ack,sizeof(ack),253)==HAL_OK);
 frame(ack,sizeof(ack),62,0);
 assert(radio_uart_enqueue_sedsnet_frame(ack,sizeof(ack),255)==HAL_OK);
 frame(ack,sizeof(ack),63,0);
 assert(radio_uart_enqueue_sedsnet_frame(ack,sizeof(ack),255)==HAL_OK);
 /* An ACK must outrank older independent routine traffic; equal priorities
  * retain FIFO order and the encoded SDT history stays intact. */
 for(unsigned marker=62;marker<=63;marker++){
  assert(radio_uart_dequeue_frame_with_budget(&out,2000));
  assert(out.data[7]==marker);g_radio_tx_ok++;
 }
 assert(radio_uart_dequeue_frame_with_budget(&out,2000)&&out.data[7]==61);g_radio_tx_ok++;
 assert(radio_uart_dequeue_frame_with_budget(&out,2000)&&out.data[7]==60);g_radio_tx_ok++;
 assert(!g_tx_count&&!g_tx_queue_bytes_used);
 /* Routine canonical heartbeats can bypass encoded history at dequeue, but
  * must not consume the two slots reserved for commands and ACKs. */
 reset(); frame(small,sizeof(small),70,0); small[6]=100;
 assert(radio_uart_independent_control(small,sizeof(small),253));
 for(unsigned i=0;i<10;i++)assert(radio_uart_enqueue_sedsnet_frame(small,sizeof(small),253)==HAL_OK);
 assert(radio_uart_enqueue_sedsnet_frame(small,sizeof(small),253)==HAL_BUSY);
 frame(schema,16,71,1);
 assert(radio_uart_enqueue_sedsnet_frame(schema,16,255)==HAL_BUSY); /* SDT cannot claim reserve. */
 frame(ack,sizeof(ack),72,0);
 assert(radio_uart_enqueue_sedsnet_frame(ack,sizeof(ack),255)==HAL_OK);
 frame(ack,sizeof(ack),73,0); ack[6]=110;
 assert(radio_uart_enqueue_sedsnet_frame(ack,sizeof(ack),254)==HAL_OK);
 assert(g_tx_count==12 && g_tx_queue_bytes_used==10*sizeof(small)+2*sizeof(ack));
 assert(radio_uart_enqueue_sedsnet_frame(ack,sizeof(ack),254)==HAL_BUSY);
 assert(radio_uart_dequeue_frame_with_budget(&out,2000)&&out.data[7]==72);g_radio_tx_ok++;
 assert(radio_uart_dequeue_frame_with_budget(&out,2000)&&out.data[7]==73);g_radio_tx_ok++;
 assert(g_tx_count==10);
 /* Byte reservation is independent of slot reservation and has the same
  * classification. Accepted ordinary bytes and SDT order remain owned. */
 reset(); frame(schema,1000,74,1);
 assert(radio_uart_enqueue_sedsnet_frame(schema,1000,253)==HAL_OK);
 frame(schema,750,75,0); schema[6]=100;
 assert(radio_uart_enqueue_sedsnet_frame(schema,750,253)==HAL_OK);
 frame(schema,60,76,0); schema[6]=100;
 assert(radio_uart_enqueue_sedsnet_frame(schema,60,253)==HAL_BUSY);
 frame(schema,60,77,1);
 assert(radio_uart_enqueue_sedsnet_frame(schema,60,255)==HAL_BUSY);
 frame(ack,sizeof(ack),78,0);
 assert(radio_uart_enqueue_sedsnet_frame(ack,sizeof(ack),255)==HAL_OK);
 frame(ack,sizeof(ack),79,0);ack[6]=110;
 assert(radio_uart_enqueue_sedsnet_frame(ack,sizeof(ack),254)==HAL_OK);
 assert(g_tx_count==4 && g_tx_queue_bytes_used==1790);
 reset(); clock_ms=1;
 radio_uart_drop_stale_locked(0); /* Keep legacy helper covered, with an empty queue. */
 frame(small,sizeof(small),41,1);
 assert(radio_uart_enqueue_sedsnet_frame(small,sizeof(small),16)==HAL_OK);
 frame(small,sizeof(small),42,1);
 assert(radio_uart_enqueue_sedsnet_frame(small,sizeof(small),16)==HAL_OK);
 clock_ms=2000;
 assert(radio_uart_tx_queue_count()==2); /* Observation cannot discard accepted SDT. */
 assert(g_tx_drops==0 && g_tx_drop_stale==0);
 for(unsigned marker=41;marker<=42;marker++){
  assert(radio_uart_dequeue_frame_with_budget(&out,2000));assert(out.data[7]==marker);g_radio_tx_ok++;
 }
 assert(radio_uart_tx_queue_count()==0);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            exe=str(Path(tmp)/'queue-test')
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',exe],
                           input=preamble+types+globals_+read+remove+stale+count+independent+enqueue+test,text=True,check=True)
            subprocess.run([exe],check=True)
