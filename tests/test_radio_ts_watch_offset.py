import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _function(source: str, signature: str) -> str:
    start = source.index(signature)
    return source[start:source.index('\n}\n', start) + 3]


class RadioTimeSyncWatchOffsetTests(unittest.TestCase):
    """With air turns RF writes a whole turn's frames in one DMA. A watched
    time sync frame is stamped when its own last byte leaves the UART (the
    edge the station stamps on receipt), not when the whole burst ends: the
    DMA completion instant minus the bytes that follow the frame in the
    burst, at the UART's 10-bit byte time."""

    def test_a_frame_early_in_a_burst_gets_its_own_departure_time(self):
        source = (ROOT / "Core/Src/radio.c").read_text()
        try:
            dma = _function(source, "static void radio_ts_watch_note_dma(")
            done = _function(source, "static void radio_ts_watch_note_complete(")
        except ValueError:
            dma = done = ""
        code = r"""#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#define RADIO_TS_WATCH_MAX 72U
static struct {
  volatile uint8_t state;
  uint8_t len;
  uint16_t tail;
  volatile uint64_t us;
  uint8_t bytes[RADIO_TS_WATCH_MAX];
} g_ts_watch;
""" + dma + done + r"""
int main(void) {
  uint8_t burst[400];
  for (unsigned i = 0; i < sizeof burst; ++i) burst[i] = (uint8_t)((i * 7 + 3) ^ ((i / 256) * 0x5A));
  /* The request is 40 bytes at offset 60 of a 400-byte turn burst. */
  memcpy(g_ts_watch.bytes, &burst[60], 40); g_ts_watch.len = 40; g_ts_watch.state = 1;
  radio_ts_watch_note_dma(burst, sizeof burst);
  assert(g_ts_watch.state == 2);
  /* The DMA ends at 1,000,000 us; 300 bytes followed the frame. At 115200
   * baud a byte is 86.8 us: the frame's last byte left 26,042 us earlier. */
  radio_ts_watch_note_complete(1000000ULL, 115200U);
  assert(g_ts_watch.state == 3);
  const uint64_t expect = 1000000ULL - (300ULL * 10ULL * 1000000ULL) / 115200ULL;
  assert(g_ts_watch.us == expect);
  /* A frame that ends the burst keeps the completion instant. */
  memcpy(g_ts_watch.bytes, &burst[360], 40); g_ts_watch.len = 40; g_ts_watch.state = 1;
  radio_ts_watch_note_dma(burst, sizeof burst);
  radio_ts_watch_note_complete(2000000ULL, 115200U);
  assert(g_ts_watch.us == 2000000ULL);
  /* No match: never stamped. */
  uint8_t other[40] = {0};
  memcpy(g_ts_watch.bytes, other, 40); g_ts_watch.state = 1;
  radio_ts_watch_note_dma(burst, sizeof burst);
  assert(g_ts_watch.state == 1);
  radio_ts_watch_note_complete(3000000ULL, 115200U);
  assert(g_ts_watch.state == 1);
  return 0;
}
"""
        self.assertTrue(dma and done, "per-frame watch functions missing: radio_ts_watch_note_dma / _complete")
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)
            (p / "t.c").write_text(code)
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined",
                            str(p / "t.c"), "-o", str(p / "t")], check=True)
            subprocess.run([str(p / "t")], check=True)


if __name__ == "__main__":
    unittest.main()
