import subprocess
import tempfile
import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
class RadioPartialFrameTests(unittest.TestCase):
    def test_corrupt_length_recovers_without_losing_following_frame(self):
        s=(ROOT/'Core/Src/radio.c').read_text()
        a=s.index('static void radio_frame_buf_consume');b=s.index('/* Drain complete frames',a)
        stub='#include <stdint.h>\n#include <stddef.h>\n#include <string.h>\n#include <assert.h>\n#include <stdio.h>\n#define RADIO_UART_FRAME_BUF_SIZE 1028\n#define RADIO_UART_FRAME_HEADER_SIZE 4\n#define RADIO_UART_MAX_PAYLOAD_SIZE 1024\n#define RADIO_UART_FRAME_SYNC_0 0xa5\n#define RADIO_UART_FRAME_SYNC_1 0x5a\n#define RADIO_UART_COMMAND_SYNC_0 0xa6\n#define RADIO_UART_COMMAND_SYNC_1 0x5b\n#define RADIO_UART_ASCII_SYNC_0 0xa7\n#define RADIO_UART_ASCII_SYNC_1 0x5c\nstatic uint8_t g_frame_buf[1028],g_current_rx_is_command_frame,g_last_frame_kind,g_last_frame_preview[16],g_last_frame_preview_len;\nstatic size_t g_frame_len;\nstatic unsigned g_rx_sync_loss,g_rx_bad_len,g_last_frame_payload_len,g_radio_rx_frames_ok,delivered;\nvoid radio_uart_store_preview(uint8_t*p,uint8_t*n,const uint8_t*d,size_t l){(void)p;(void)n;(void)d;(void)l;}\nvoid radio_notify_rx(const uint8_t*d,size_t l){(void)d;(void)l;delivered++;}\n\nstatic uint32_t mock_now,g_partial_frame_started_ms; static uint8_t g_partial_frame_waiting;\n#define RADIO_PARTIAL_FRAME_TIMEOUT_MS 2000U\nuint32_t radio_now_ms(void){return mock_now;}\n'
        main=r"""
int main(void){
 const uint8_t bad[]={0xa5,0x5a,0,4},good[]={0xa5,0x5a,3,0,1,2,3};
 mock_now=0xfffffff0U;
 radio_frame_buf_append(bad,4);radio_process_buffered_frames();
 radio_frame_buf_append(good,7);radio_process_buffered_frames();assert(delivered==0);
 mock_now+=1999;radio_process_buffered_frames();assert(delivered==0);
 mock_now+=1;radio_process_buffered_frames();assert(delivered==1&&g_frame_len==0);
 // A legitimate split frame arriving within the deadline is preserved.
 delivered=0;radio_frame_buf_append(good,5);radio_process_buffered_frames();
 mock_now+=1999;radio_frame_buf_append(good+5,2);radio_process_buffered_frames();
 assert(delivered==1&&g_rx_bad_len==1);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'test.c').write_text(stub+s[a:b]+main)
            subprocess.run(['cc','-std=c11','-fsanitize=address,undefined',str(p/'test.c'),'-o',str(p/'test')],check=True)
            subprocess.run([str(p/'test')],check=True)
