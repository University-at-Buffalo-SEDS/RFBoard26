import re
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class RadioAirBudgetTests(unittest.TestCase):
    def test_pacing_and_tick_wrap(self):
        source = (ROOT / "Core/Src/radio.c").read_text()
        funcs = source[source.index("static uint32_t radio_uart_air_ms(uint16_t len)\n{"):source.index("uint8_t radio_uart_tx_ready(")]
        preamble = """#include <stdint.h>
#include <assert.h>
#define RADIO_AIR_BIT_RATE_BPS 64000U
#define RADIO_AIR_SHARE_PERCENT 40U
#define RADIO_AIR_FRAME_OVERHEAD_BYTES 16U
#define RADIO_TX_COOLDOWN_MS 0U
static uint32_t now, g_tx_quiet_until_ms, g_tx_air_budget_active;
static uint32_t radio_now_ms(void) { return now; }
"""
        test = """int main(void) {
assert(!radio_uart_air_busy());
assert(radio_uart_air_ms(104)==38);
assert(radio_uart_air_ms(1028)==327);
now=UINT32_MAX-10; radio_uart_mark_tx_quiet(104);
assert(radio_uart_air_busy()); now=20; assert(radio_uart_air_busy());
now=27; assert(!radio_uart_air_busy());
now=UINT32_MAX; assert(!radio_uart_air_busy());
}
"""
        with tempfile.TemporaryDirectory() as directory:
            exe = str(Path(directory) / "test")
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined", "-x", "c", "-", "-o", exe], input=preamble+funcs+test, text=True, check=True)
            subprocess.run([exe], check=True)

    def test_service_waits_before_selecting_next_priority_frame(self):
        source = (ROOT / "Core/Src/radio.c").read_text()
        service = source.split("uint32_t radio_uart_process_tx_with_budget", 1)[1].split("HAL_StatusTypeDef radio_uart_subscribe_rx", 1)[0]
        self.assertLess(service.index("if (radio_uart_air_busy()) return 0U;"), service.index("radio_uart_dequeue_frame_with_budget"))
        self.assertIn("radio_uart_mark_tx_quiet(g_tx_dma_item.len);", service)
        self.assertNotIn("tx_thread_sleep", service)
