"""Retained first sync bytes are normal UART fragmentation, not lost framing."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def parser_program():
    source = (ROOT / 'Core/Src/radio.c').read_text()
    parser = source[source.index('static void radio_frame_buf_consume'):source.index('/* Push one RX chunk')]
    return f'#include "{ROOT / "Core/Inc/recovery_mesh.h"}"\n#include "{ROOT / "tests/radio_crc_reference.h"}"\n' + r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
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
#define RADIO_PARTIAL_FRAME_TIMEOUT_MS 2000U
static uint8_t g_frame_buf[1028], g_current_rx_is_command_frame, g_last_frame_kind;
static uint8_t g_last_frame_preview[16], g_last_frame_preview_len, g_partial_frame_waiting;
static size_t g_frame_len, g_last_frame_payload_len;
static uint32_t g_partial_frame_started_ms, g_rx_sync_loss, g_rx_bad_len, g_radio_rx_frames_ok, received;
static uint32_t radio_now_ms(void) { return 0; }
static void radio_uart_store_preview(uint8_t*a,uint8_t*b,const uint8_t*c,size_t n) { (void)a;(void)b;(void)c;(void)n; }
static void radio_notify_rx(const uint8_t*p,size_t n) { (void)p;(void)n; received++; }
''' + parser


class SyncLossCounterTests(unittest.TestCase):
    def test_retained_prefix_is_not_loss_but_discarded_junk_is(self):
        main = r'''
int main(void) {
 for (unsigned mode=0; mode<3; ++mode) {
  const uint8_t starts[]={0xa5,0xa6,0xa7}, ends[]={0x5a,0x5b,0x7a};
  g_frame_len=0; g_rx_sync_loss=0; g_radio_rx_frames_ok=0;
  radio_process_framed_bytes(&starts[mode],1);
  assert(g_frame_len==1 && g_rx_sync_loss==0);
  const uint8_t rest[]={ends[mode],1,0,42};
  radio_process_framed_bytes(rest,sizeof(rest));
  assert(g_frame_len==0 && g_radio_rx_frames_ok==1 && g_rx_sync_loss==0);
 }
 const uint8_t junk[]={0,1,2};
 radio_process_framed_bytes(junk,sizeof(junk));
 assert(g_frame_len==0 && g_rx_sync_loss==1);
 const uint8_t junk_then_prefix[]={0,1,0xa5};
 radio_process_framed_bytes(junk_then_prefix,sizeof(junk_then_prefix));
 assert(g_frame_len==1 && g_rx_sync_loss==2);
 const uint8_t rest[]={0x5a,1,0,42};
 radio_process_framed_bytes(rest,sizeof(rest));
 assert(g_frame_len==0 && g_rx_sync_loss==2);
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / 'sync-counter'
            compiled = subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(binary)], input=parser_program()+main, text=True, capture_output=True)
            self.assertEqual(compiled.returncode,0,compiled.stderr)
            subprocess.run([str(binary)],check=True)
