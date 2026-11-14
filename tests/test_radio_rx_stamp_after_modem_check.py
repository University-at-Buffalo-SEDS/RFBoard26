"""A received frame's stamp must not move after the startup modem check.

The RX event ring records the ISR's received-byte total (every byte the UART
DMA delivered, the modem check's AT replies included); a frame's end is
located by the count of bytes taken out of the RX ring. The modem check reads
its replies straight from the ring and the hold clears it, so unless those
bytes count as taken too, every later frame's end falls hundreds of bytes
before the events that really cover it: the stamp lands on an earlier event,
tens of ms early (581 AT bytes on the shipped profile = 50 ms), and the time
sync round trip looks shorter than the radio's physical minimum."""
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _function(source: str, signature: str) -> str:
    start = source.index(signature)
    return source[start:source.index('\n}\n', start) + 3]


class RadioRxStampAfterModemCheckTests(unittest.TestCase):
    def test_modem_check_bytes_keep_frame_ends_in_the_event_domain(self):
        source = (ROOT / 'Core/Src/radio.c').read_text()
        item = re.search(r'typedef struct \{[^}]*\} radio_rx_item_t;', source).group()
        metadata = source.split('/* ISR -> thread ring buffer */', 1)[1].split('static uint8_t g_rx_ring', 1)[0]
        start = source.index('static inline void radio_rx_ring_push_isr(')
        ring_fns = source[start:source.index('/**\n * Call this periodically', start)]
        parts = [
            _function(source, 'static void radio_rx_note_event('),
            _function(source, 'static uint64_t radio_rx_frame_us('),
            ring_fns,
            _function(source, 'static void radio_rx_ring_clear('),
            _function(source, 'int radio_uart_modem_read('),
        ]
        harness = r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#define RADIO_UART_RX_BUF_SIZE 256U
#define RADIO_UART_RX_RING_DEPTH 8U
#define RADIO_RX_EVENTS 8U
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
static uint32_t g_rx_isr_drops, g_rx_isr_chunks, g_rx_isr_bytes;
static struct { uint32_t bytes; uint64_t us; } g_rx_events[RADIO_RX_EVENTS];
static uint8_t g_rx_event_head;
static uint32_t g_rx_appended_bytes;
static uint8_t g_radio_modem_hold;
static void radio_uart_recover_rx_if_needed(void) {}
'''
        test = r'''
/* What the ISR does per RX event: push the chunk, then note the total. */
static void rx_event(const uint8_t *d, uint16_t n, uint64_t us) {
  radio_rx_ring_push_isr(d, n);
  radio_rx_note_event(g_rx_isr_bytes, us);
}
/* What the thread does: every popped byte goes to the framer. */
static void drain_to_framer(void) {
  radio_rx_item_t it;
  while (radio_rx_ring_pop_thread(&it)) g_rx_appended_bytes += it.len;
}
int main(void) {
  uint8_t at[200]; memset(at, 'A', sizeof(at));
  uint8_t frame[62]; memset(frame, 0x5A, sizeof(frame));
  uint8_t out[64];
  /* Startup modem check: 581 bytes of AT replies read by the check. */
  g_radio_modem_hold = 1U;
  rx_event(at, 200, 1000000ULL); while (radio_uart_modem_read(out, sizeof(out)) > 0) {}
  rx_event(at, 200, 1020000ULL); while (radio_uart_modem_read(out, sizeof(out)) > 0) {}
  rx_event(at, 150, 1040000ULL); while (radio_uart_modem_read(out, sizeof(out)) > 0) {}
  /* Leftover the hold's ring clear discards. */
  rx_event(at, 31, 1050000ULL);
  radio_rx_ring_clear();
  g_radio_modem_hold = 0U;
  /* A time sync response alone on the line, 1.95 s later. */
  rx_event(frame, 62, 3000000ULL);
  drain_to_framer();
  const uint64_t us = radio_rx_frame_us(g_rx_appended_bytes, 115200U);
  assert(us == 3000000ULL);
  /* Two frames in one write: the first ends 62 bytes before the event. */
  rx_event(frame, 62, 3100000ULL);
  rx_event(frame, 62, 3200000ULL);
  drain_to_framer();
  assert(radio_rx_frame_us(g_rx_appended_bytes - 62U, 115200U) == 3100000ULL);
  assert(radio_rx_frame_us(g_rx_appended_bytes, 115200U) == 3200000ULL);
  return 0;
}
'''
        code = harness + metadata + 'static uint8_t g_rx_ring[RADIO_UART_RX_RING_BYTES];\n' + item + '\n' + '\n'.join(parts) + test
        with tempfile.TemporaryDirectory() as d:
            exe = str(Path(d) / 't')
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                            '-fsanitize=address,undefined', '-x', 'c', '-', '-o', exe],
                           input=code, text=True, check=True)
            subprocess.run([exe], check=True)


if __name__ == '__main__':
    unittest.main()
