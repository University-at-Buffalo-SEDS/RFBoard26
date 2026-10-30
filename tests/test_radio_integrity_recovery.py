"""Exercise the production UART parser after an interrupted physical write."""
from pathlib import Path
import subprocess
import tempfile
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]


def frame(body, command=False):
    return (b"\xa6\x5b" if command else b"\xa5\x5a") + len(body).to_bytes(2, "little") + body


def relay(body, kind):
    encoded = b"RR\x01" + bytes([kind]) + body
    return encoded + zlib.crc32(encoded).to_bytes(4, "little")


class RadioIntegrityRecovery(unittest.TestCase):
    def test_interrupted_canonical_write_preserves_the_exact_retry(self):
        self._exercise_interrupted_writes(False)

    def test_interrupted_relay_write_preserves_the_exact_retry(self):
        self._exercise_interrupted_writes(True)

    def _exercise_interrupted_writes(self, relayed):
        # Canonical DiscoveryAddress from the actual SDK-decoded radio fixture;
        # its public RF identity contains no credentials or application outputs.
        canonical = bytes.fromhex(
            "4a01115104b10bdbfcacf20e8400020000959afadf000000000100000000000000"
            "65422c5af1d8d68502000000524601000000f81ab10a020000006cab1104b82b4d0f"
            "01000000020000005246011300000002c000000028000000100000003cad6ee6"
        )
        sdt = (ROOT / "tests/fixtures/radio-startup-discovery-frame.bin").read_bytes()[4:]
        for body in (canonical, sdt):
            self.assertEqual(zlib.crc32(body[:-4]), int.from_bytes(body[-4:], "little"))
        vectors = [
            ("relay_data", frame(relay(canonical, 5), True), canonical, 0),
            ("relay_command", frame(relay(canonical, 6), True), canonical, 1),
            ("relay_sdt", frame(relay(sdt, 5), True), sdt, 0),
        ]
        if not relayed:
            vectors = [("canonical", frame(canonical), canonical, 0)]
        source = (ROOT / "Core/Src/radio.c").read_text()
        parser = source[source.index("static void radio_frame_buf_consume"):
                        source.index("/* Push one RX chunk")]
        declarations = []
        cases = []
        for name, wire, body, command in vectors:
            for suffix, data in (("wire", wire), ("body", body)):
                declarations.append(f"static const uint8_t {name}_{suffix}[]={{" +
                                    ",".join(map(str, data)) + "};")
            cases.append(f'CHECK("{name}", {name}_wire, {name}_body, {command});')
        stub = r'''
#include <assert.h>
#include <stdio.h>
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
static uint32_t g_partial_frame_started_ms,g_rx_sync_loss,g_rx_bad_len,g_radio_rx_frames_ok;
static uint32_t received,unexpected,mock_now;
static const uint8_t *expected;
static uint8_t expected_command;
static uint32_t radio_now_ms(void){return mock_now;}
static void radio_uart_store_preview(uint8_t*a,uint8_t*b,const uint8_t*c,size_t n){(void)a;(void)b;(void)c;(void)n;}
static void radio_notify_rx(const uint8_t*p,size_t n){
 if(n==expected_length && memcmp(p,expected,n)==0 &&
    g_current_rx_is_command_frame==expected_command) ++received;
 else ++unexpected;
}
'''
        main = r'''
#define CHECK(label, wire, body, command) do { \
 const size_t chunks[]={1U,2U,7U,31U,sizeof(wire),2U*sizeof(wire)}; \
 for(size_t prefix=1U;prefix<sizeof(wire);++prefix) { \
  for(size_t c=0;c<sizeof(chunks)/sizeof(chunks[0]);++c) { \
   uint8_t input[2U*sizeof(wire)]; \
   memcpy(input,wire,prefix);memcpy(input+prefix,wire,sizeof(wire)); \
   g_frame_len=0;g_partial_frame_waiting=0;received=0;unexpected=0;mock_now=0; \
   expected=body;expected_length=sizeof(body);expected_command=command; \
   const size_t length=prefix+sizeof(wire); \
   for(size_t offset=0;offset<length;) { \
    size_t count=length-offset;if(count>chunks[c])count=chunks[c]; \
    radio_process_framed_bytes(input+offset,count);offset+=count; \
   } \
   mock_now+=RADIO_PARTIAL_FRAME_TIMEOUT_MS;radio_process_buffered_frames(); \
   /* A retry byte can complete the interrupted frame's identical CRC. The \
    * remaining suffix may then leave an ordinary incomplete sync header. */ \
   const int valid_tail = g_frame_len == 0U || (g_frame_len <= 3U && \
    (g_frame_buf[0] == RADIO_UART_FRAME_SYNC_0 || \
     g_frame_buf[0] == RADIO_UART_COMMAND_SYNC_0 || \
     g_frame_buf[0] == RADIO_UART_ASCII_SYNC_0)); \
   if(received!=1U || unexpected!=0U || !valid_tail) { \
    fprintf(stderr,"%s prefix=%zu chunk=%zu valid=%u invalid=%u pending=%zu\n", \
      label,prefix,chunks[c],received,unexpected,g_frame_len);return 2; \
   } \
   radio_process_framed_bytes(wire,sizeof(wire)); \
   mock_now+=RADIO_PARTIAL_FRAME_TIMEOUT_MS;radio_process_buffered_frames(); \
   if(received!=2U || unexpected!=0U) { \
    fprintf(stderr,"%s subsequent retry lost after prefix=%zu chunk=%zu\n", \
      label,prefix,chunks[c]);return 3; \
   } \
  } \
 } \
} while(0)
int main(void) {
'''
        program = (
            f'#include "{ROOT / "Core/Inc/recovery_mesh.h"}"\n'
            f'#include "{ROOT / "tests/radio_crc_reference.h"}"\n'
            + "\n".join(declarations) + stub + parser + main
            + "\n".join(cases) + "\nreturn 0;}\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "radio-integrity"
            compiled = subprocess.run(
                ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                 "-fsanitize=address,undefined", "-x", "c", "-", "-o", str(executable)],
                input=program, text=True, capture_output=True,
            )
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    unittest.main()
