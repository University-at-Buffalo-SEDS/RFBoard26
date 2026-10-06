"""Exercise the production framing parser with arbitrary UART chunk boundaries."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]
class RadioFramingTests(unittest.TestCase):
    def test_back_to_back_maximum_frames(self):
        source = (ROOT / 'Core/Src/radio.c').read_text()
        parser = source[source.index('static void radio_frame_buf_consume'):source.index('/* Push one RX chunk')]
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define RADIO_UART_FRAME_BUF_SIZE 1028U
#define RADIO_UART_MAX_PAYLOAD_SIZE 1024U
#define RADIO_UART_FRAME_HEADER_SIZE 4U
#define RADIO_UART_FRAME_SYNC_0 0xA5
#define RADIO_UART_FRAME_SYNC_1 0x5A
#define RADIO_UART_COMMAND_SYNC_0 0xA6
#define RADIO_UART_COMMAND_SYNC_1 0x5B
#define RADIO_UART_ASCII_SYNC_0 0xA7
#define RADIO_UART_ASCII_SYNC_1 0x7A
static uint8_t g_frame_buf[1028],g_current_rx_is_command_frame,g_last_frame_kind;
static uint8_t g_last_frame_preview[16],g_last_frame_preview_len;
static size_t g_frame_len,g_last_frame_payload_len;
#define RADIO_PARTIAL_FRAME_TIMEOUT_MS 2000U
static uint32_t g_partial_frame_started_ms;
static uint8_t g_partial_frame_waiting;
static uint32_t radio_now_ms(void){return 0U;}
static unsigned g_rx_sync_loss,g_rx_bad_len,g_radio_rx_frames_ok,received;
static void radio_uart_store_preview(uint8_t *a,uint8_t *b,const uint8_t *c,size_t n){(void)a;(void)b;(void)c;(void)n;}
static void radio_notify_rx(const uint8_t *p,size_t n){
    assert(n==(received%2==0?1024U:37U));
    for(size_t i=0;i<n;i++) assert(p[i]==(uint8_t)(i+received));
    received++;
}
''' + parser + r'''
int main(void){
    uint8_t wire[5000];size_t n=0;
    for(unsigned f=0;f<4;f++){
        size_t len=f%2==0?1024:37;
        wire[n++]=0xA5;wire[n++]=0x5A;wire[n++]=len;wire[n++]=len>>8;
        for(size_t i=0;i<len;i++)wire[n++]=(uint8_t)(i+f);
    }
    for(size_t chunk=1;chunk<=1028;chunk++){
        received=0;g_frame_len=0;g_radio_rx_frames_ok=0;
        for(size_t off=0;off<n;){size_t take=n-off<chunk?n-off:chunk;
            radio_process_framed_bytes(wire+off,take);off+=take;}
        assert(received==4 && g_frame_len==0 && g_radio_rx_frames_ok==4);
    }
}
'''
        with tempfile.TemporaryDirectory() as d:
            exe=Path(d)/'framing'
            p=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=code,text=True,capture_output=True)
            self.assertEqual(p.returncode,0,p.stderr)
            subprocess.run([str(exe)],check=True)
