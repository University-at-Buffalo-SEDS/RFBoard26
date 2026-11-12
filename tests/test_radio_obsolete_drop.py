import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class RadioObsoleteDropTests(unittest.TestCase):
    """A relayed reliable copy that every owner already acknowledged is
    dropped from the radio queue; controls and other frames stay."""

    def test_drops_only_obsolete_small_ordinary_frames(self):
        source = (ROOT / "Core/Src/radio.c").read_text()
        start = source.index("static radio_frame_obsolete_cb_t g_obsolete_check;")
        end = source.index("void radio_uart_watch_frame(")
        funcs = source[start:end]
        preamble = r"""#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
#define RADIO_UART_FRAME_HEADER_SIZE 4U
#define RADIO_UART_PRIORITY_NETWORK_CONTROL 254U
typedef uint8_t (*radio_frame_obsolete_cb_t)(const uint8_t *frame, size_t len);
typedef struct { uint16_t len; uint8_t uart_frames; uint16_t offset; uint8_t priority; uint32_t enqueued_ms; } radio_tx_queue_item_t;
static radio_tx_queue_item_t g_tx_queue[8];
static uint8_t g_tx_queue_bytes[512];
static uint32_t g_tx_count;
static int locked;
static HAL_StatusTypeDef radio_uart_lock_tx_queue(void) { assert(!locked); locked = 1; return HAL_OK; }
static void radio_uart_unlock_tx_queue(void) { assert(locked); locked = 0; }
static void radio_uart_remove_queue_index(uint32_t i) {
  memmove(&g_tx_queue[i], &g_tx_queue[i + 1], (g_tx_count - i - 1) * sizeof(g_tx_queue[0]));
  g_tx_count--;
}
static void push(uint8_t prio, uint8_t tag, uint16_t payload) {
  radio_tx_queue_item_t *it = &g_tx_queue[g_tx_count];
  it->len = (uint16_t)(payload + 4U); it->uart_frames = 1U; it->priority = prio;
  it->offset = (uint16_t)(g_tx_count * 64U); it->enqueued_ms = g_tx_count;
  memset(&g_tx_queue_bytes[it->offset], 0, it->len);
  g_tx_queue_bytes[it->offset + 4U] = tag;
  g_tx_count++;
}
static int calls;
static uint8_t obsolete(const uint8_t *frame, size_t len) {
  assert(!locked);           /* never called with the queue locked */
  calls++;
  assert(len == 20U);        /* header stripped */
  return frame[0] == 0xA1;   /* the acknowledged battery copy */
}
"""
        test = r"""int main(void) {
push(16U, 0xA1, 20U);   /* acknowledged relayed copy: dropped */
push(254U, 0xA1, 20U);  /* network control: never checked */
push(16U, 0xB2, 20U);   /* still needed */
push(255U, 0xA1, 20U);  /* urgent (ABORT): never checked */
assert(radio_uart_drop_obsolete() == 0U);  /* no check installed */
radio_uart_set_obsolete_check(obsolete);
assert(radio_uart_drop_obsolete() == 1U);
assert(calls == 2);
assert(g_tx_count == 3U);
assert(g_tx_queue[0].priority == 254U && g_tx_queue[1].priority == 16U && g_tx_queue[2].priority == 255U);
assert(g_tx_queue_bytes[g_tx_queue[1].offset + 4U] == 0xB2);
assert(g_radio_tx_obsolete_drops == 1U);
return 0;
}
"""
        with tempfile.TemporaryDirectory() as directory:
            exe = str(Path(directory) / "test")
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined",
                            "-x", "c", "-", "-o", exe], input=preamble + funcs + test, text=True, check=True)
            subprocess.run([exe], check=True)


if __name__ == "__main__":
    unittest.main()
