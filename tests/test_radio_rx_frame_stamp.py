import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _function(source: str, signature: str) -> str:
    start = source.index(signature)
    return source[start:source.index('\n}\n', start) + 3]


class RadioRxFrameStampTests(unittest.TestCase):
    """The station writes several frames at once (HEALTH, a response, ACKs),
    and the UART RX event comes when the line goes idle after all of them.
    A received frame is stamped when its own last byte arrived: the first
    RX event whose byte count covers the frame's end, minus the bytes that
    came in after it at the UART byte time."""

    def test_each_frame_is_stamped_at_its_own_end(self):
        source = (ROOT / "Core/Src/radio.c").read_text()
        try:
            note = _function(source, "static void radio_rx_note_event(")
            frame = _function(source, "static uint64_t radio_rx_frame_us(")
        except ValueError:
            note = frame = ""
        self.assertTrue(note and frame, "radio_rx_note_event / radio_rx_frame_us missing")
        code = r"""#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#define RADIO_RX_EVENTS 8U
static struct { uint32_t bytes; uint64_t us; } g_rx_events[RADIO_RX_EVENTS];
static uint8_t g_rx_event_head;
""" + note + frame + r"""
int main(void) {
  const uint64_t byte_us_x = 10ULL * 1000000ULL; /* / baud */
  radio_rx_note_event(1000U, 1000000ULL);
  /* 100 bytes followed the frame before that idle event. */
  assert(radio_rx_frame_us(900U, 115200U) == 1000000ULL - (100ULL * byte_us_x) / 115200ULL);
  /* The last frame of the write: the event instant itself. */
  assert(radio_rx_frame_us(1000U, 115200U) == 1000000ULL);
  /* A later event after a quiet gap: an earlier frame keeps its own event. */
  radio_rx_note_event(1500U, 2000000ULL);
  assert(radio_rx_frame_us(950U, 115200U) == 1000000ULL - (50ULL * byte_us_x) / 115200ULL);
  assert(radio_rx_frame_us(1100U, 115200U) == 2000000ULL - (400ULL * byte_us_x) / 115200ULL);
  /* Byte counters wrap. */
  radio_rx_note_event(0x00000010U, 3000000ULL);
  assert(radio_rx_frame_us(0xFFFFFFF0U, 115200U) == 3000000ULL - (32ULL * byte_us_x) / 115200ULL);
  return 0;
}
"""
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)
            (p / "t.c").write_text(code)
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined",
                            str(p / "t.c"), "-o", str(p / "t")], check=True)
            subprocess.run([str(p / "t")], check=True)


if __name__ == "__main__":
    unittest.main()
