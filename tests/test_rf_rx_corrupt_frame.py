"""Preserve the valid frame following the corruption captured over FTDI."""
import binascii
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class CorruptRadioFrameTests(unittest.TestCase):
    def test_truncated_sdt_frame_cannot_consume_a_following_frame(self):
        source=(ROOT/'Core/Src/radio.c').read_text()
        parser=source[source.index('static void radio_frame_buf_consume'):source.index('/* Push one RX chunk')]
        # Use the existing real SDK frame rather than an unchecked byte pattern.
        good=(ROOT/'tests/fixtures/radio-startup-discovery-frame.bin').read_bytes()[4:]
        valid_body=b'SDT\x03'+b'1'*184
        valid=valid_body+binascii.crc32(valid_body).to_bytes(4,'little')
        damaged=b'\xa5\x5a\xc0\x00'+valid[:162]+b'\xa5\x5a'+len(good).to_bytes(2,'little')+good
        intact=b'\xa5\x5a\xc0\x00'+valid
        arrays=''.join('static const uint8_t '+name+'[]={'+','.join(map(str,data))+'};\n' for name,data in [('good',good),('valid',valid),('damaged',damaged),('intact',intact)])
        stub=r"""
#include <assert.h>
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
static uint8_t g_frame_buf[1028],g_current_rx_is_command_frame,g_last_frame_kind;
static uint8_t g_last_frame_preview[16],g_last_frame_preview_len,g_partial_frame_waiting;
static size_t g_frame_len,g_last_frame_payload_len,expected_length;
static uint32_t g_partial_frame_started_ms,g_rx_sync_loss,g_rx_bad_len,g_radio_rx_frames_ok,received;
static const uint8_t *expected;
static uint32_t radio_now_ms(void){return 0;}
static void radio_uart_store_preview(uint8_t*a,uint8_t*b,const uint8_t*c,size_t n){(void)a;(void)b;(void)c;(void)n;}
static void radio_notify_rx(const uint8_t*p,size_t n){assert(n==expected_length);assert(memcmp(p,expected,n)==0);received++;}
"""
        main=r"""
int main(void){
 assert(launchcore_crc32("123456789",9)==0xcbf43926U);
 for(size_t chunk=1;chunk<=sizeof(damaged);chunk++){
  g_frame_len=0;g_partial_frame_waiting=0;g_rx_crc_errors=0;received=0;
  expected=good;expected_length=sizeof(good);
  for(size_t off=0;off<sizeof(damaged);){size_t n=sizeof(damaged)-off;if(n>chunk)n=chunk;radio_process_framed_bytes(damaged+off,n);off+=n;}
  assert(received==1&&g_frame_len==0&&g_rx_crc_errors==1);
 }
 expected=valid;expected_length=sizeof(valid);received=0;
 radio_process_framed_bytes(intact,sizeof(intact));assert(received==1&&g_frame_len==0);
}
"""
        includes=f'#include "{ROOT / "Core/Inc/recovery_mesh.h"}"\n#include "{ROOT / "tests/radio_crc_reference.h"}"\n'
        with tempfile.TemporaryDirectory() as d:
            exe=Path(d)/'corrupt-frame'
            compiled=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=includes+arrays+stub+parser+main,text=True,capture_output=True)
            self.assertEqual(compiled.returncode,0,compiled.stderr)
            subprocess.run([str(exe)],check=True)
