import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class BoardClockUsTests(unittest.TestCase):
    """board_now_us(): ThreadX tick + SysTick within the tick, including a
    reload whose tick interrupt is still pending; CAN 24-bit receive stamps
    reconstruct across the 24-bit wrap."""

    def run_c(self, body):
        header = (ROOT / "Core/Inc/board_clock_us.h").read_text()
        header = header.replace('#include "main.h"\n', "").replace('#include "tx_api.h"\n', "")
        mocks = r"""#include <stdint.h>
#include <assert.h>
#define TX_TIMER_TICKS_PER_SECOND 1000U
static uint32_t primask_v, ticks_v, val_seq[2], val_i;
static uint32_t __get_PRIMASK(void) { return primask_v; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t v) { primask_v = v; }
static uint32_t tx_time_get(void) { return ticks_v; }
typedef struct { uint32_t VAL_dummy; uint32_t LOAD; } systick_t;
static uint32_t read_val(void) { return val_seq[val_i < 1 ? val_i++ : 1]; }
static systick_t systick = { 0, 169999U };
#define SysTick_VAL read_val()
struct { uint32_t ICSR; } scb;
#define SCB (&scb)
#define SCB_ICSR_PENDSTSET_Msk (1UL << 26)
"""
        header = header.replace("SysTick->VAL", "SysTick_VAL").replace("SysTick->LOAD", "systick.LOAD")
        src = mocks + header + body
        with tempfile.TemporaryDirectory() as d:
            exe = str(Path(d) / "t")
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                            "-fsanitize=address,undefined", "-x", "c", "-", "-o", exe],
                           input=src, text=True, check=True)
            subprocess.run([exe], check=True)

    def test_sub_tick_and_pending_reload(self):
        self.run_c(r"""int main(void) {
  /* 1234 ms, counter at mid tick (170 MHz, counts down from 169999). */
  ticks_v = 1234U; val_seq[0] = 85000U; val_seq[1] = 85000U; val_i = 0; scb.ICSR = 0;
  assert(board_now_us() == 1234499U);
  /* Reloaded, tick interrupt pending: the second read is just after reload. */
  ticks_v = 1234U; val_seq[0] = 3U; val_seq[1] = 169990U; val_i = 0; scb.ICSR = 1UL << 26;
  uint64_t a = board_now_us();
  assert(a >= 1235000U && a < 1235001U);
  /* Monotonic across the boundary once the tick interrupt ran. */
  ticks_v = 1235U; val_seq[0] = 169980U; val_seq[1] = 169980U; val_i = 0; scb.ICSR = 0;
  assert(board_now_us() >= a);
  return 0;
}
""")

    def test_can_stamp_wrap(self):
        src = (ROOT / "Core/Src/can_bus.c").read_text()
        self.assertIn("uint8_t ts[3];", src)
        self.assertIn("& 0xFFFFFFU", src)
        self.run_c(r"""int main(void) {
  /* receive stamp 0xFFFFF0 (low 24 bits), delivered 0x20 us later after the 24-bit wrap */
  const uint64_t now = 0x13000010ULL;
  const uint32_t ts = 0xFFFFF0U;
  const uint32_t age = ((uint32_t)now - ts) & 0xFFFFFFU;
  assert(age == 0x20U && now - age == 0x12FFFFF0ULL);
  return 0;
}
""")


if __name__ == "__main__":
    unittest.main()
