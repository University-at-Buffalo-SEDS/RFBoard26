"""Actual RF queue/ACK timing counters reported in BOARD_HEALTH words 16..21."""
from pathlib import Path
import re
import unittest

import test_radio_control_fifo as fifo
from test_radio_funded_packet_boundary import ACK_FIXTURES, function

ROOT = Path(__file__).resolve().parents[1]


class RadioTxTimingHealthTests(unittest.TestCase):
    def test_ack_enqueue_to_dma_complete_window_high_water_and_stale_drops(self):
        source = (ROOT / 'Core/Src/radio.c').read_text()
        frames = '\n'.join(re.search(r'static const uint8_t ' + name + r'\[\]=\{[^;]+;', ACK_FIXTURES).group()
                           for name in ['und', 'fc'])
        timing = source[source.index('#define RADIO_TX_TIMING_HEALTH_TAG'):]
        code = frames + r'''
static volatile uint32_t g_tx_ack_wait_frames, g_tx_ack_wait_sum_ms, g_tx_ack_wait_max_ms;
''' + function(source, 'static void radio_uart_note_ack_completion(void)') + '\n' + timing + r'''
static uint32_t w[6];
static void complete(const radio_tx_item_t *item, uint32_t at){g_tx_dma_item=*item;clock_ms=at;radio_uart_note_ack_completion();}
int main(void){
 radio_tx_item_t out;
 uint8_t sdt[16]={165,90,12,0,'S','D','T',3,0x11};
 /* Two canonical ACKs enqueued 20 ms apart leave in one DMA 100 ms after the first. */
 clock_ms=10;assert(radio_uart_enqueue_sedsnet_parts(und,sizeof(und),NULL,0U,255)==HAL_OK);
 clock_ms=30;assert(radio_uart_enqueue_sedsnet_parts(fc,sizeof(fc),NULL,0U,255)==HAL_OK);
 assert(radio_uart_dequeue_frame_with_budget(&out,2000));
 assert(out.uart_frames==2&&g_tx_dma_ack_frames==2&&g_tx_dma_ack_offset_ms==20);
 complete(&out,110);
 assert(!g_tx_dma_ack_frames);
 radio_uart_note_ack_completion(); /* a repeated callback cannot count twice */
 board_health_radio_tx_timing(w);
 assert(w[0]==0x31544652U&&w[1]==2&&w[2]==90&&w[3]==100&&w[4]==2&&w[5]==0);
 /* Window restarts; high-water restarts at the current depth. */
 board_health_radio_tx_timing(w);
 assert(w[1]==0&&w[2]==0&&w[3]==0&&w[4]==0);
 /* Non-ACK traffic is not an ACK sample but still counts toward depth. */
 clock_ms=200;assert(radio_uart_enqueue_sedsnet_parts(sdt,sizeof(sdt),NULL,0U,253)==HAL_OK);
 assert(radio_uart_enqueue_sedsnet_parts(und,sizeof(und),NULL,0U,255)==HAL_OK);
 sdt[8]=0x22;assert(radio_uart_enqueue_sedsnet_parts(sdt,sizeof(sdt),NULL,0U,253)==HAL_OK);
 assert(radio_uart_dequeue_frame_with_budget(&out,2000)&&out.len==sizeof(und)&&g_tx_dma_ack_frames==1&&!g_tx_dma_ack_offset_ms);
 complete(&out,207);
 assert(radio_uart_dequeue_frame_with_budget(&out,2000)&&memcmp(out.data+4,"SDT",3)==0&&!g_tx_dma_ack_frames);
 assert(out.uart_frames==2&&!g_tx_count); /* ordinary SDT batch */
 complete(&out,300);
 board_health_radio_tx_timing(w);
 assert(w[1]==1&&w[2]==7&&w[3]==7&&w[4]==3&&w[5]==0);
 /* Ordinary traffic older than the age limit is a stale drop; cumulative. */
 clock_ms=1000;assert(radio_uart_enqueue_sedsnet_parts(sdt,sizeof(sdt),NULL,0U,253)==HAL_OK);
 board_health_radio_tx_timing(w);assert(w[4]==1);
 board_health_radio_tx_timing(w);assert(w[4]==1); /* still-queued depth carries into the next window */
 clock_ms=1000+1500;radio_uart_drop_stale_locked(clock_ms);
 assert(!g_tx_count);
 board_health_radio_tx_timing(w);
 assert(w[1]==0&&w[4]==1&&w[5]==1);
 board_health_radio_tx_timing(w);
 assert(w[4]==0&&w[5]==1);
 /* The 32-bit millisecond clock may wrap between enqueue and completion. */
 clock_ms=UINT32_MAX-4;assert(radio_uart_enqueue_sedsnet_parts(fc,sizeof(fc),NULL,0U,255)==HAL_OK);
 assert(radio_uart_dequeue_frame_with_budget(&out,2000)&&g_tx_dma_ack_frames==1);
 complete(&out,5);
 board_health_radio_tx_timing(w);
 assert(w[1]==1&&w[2]==10&&w[3]==10);
 return 0;
}
'''
        fifo.RadioControlFifoTests()._run_case(code)


if __name__ == '__main__':
    unittest.main()
