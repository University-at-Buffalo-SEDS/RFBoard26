"""Run the production modem gates for RFD and explicit legacy LoRa builds."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class ModemSchedulingTests(unittest.TestCase):
    def test_rfd_is_immediately_ready_and_lora_retains_its_gates(self):
        source = (ROOT / 'Core/Src/radio.c').read_text()
        ready_start = source.index('static uint8_t radio_e22_ready_for_uart(void)\n{')
        ready_end = source.index('\n}\n', ready_start) + 3
        gates_start = source.index('static uint32_t radio_uart_air_ms(uint16_t len)\n{')
        gates_end = source.index('uint8_t radio_uart_tx_ready(void)', gates_start)
        program = r'''
#include <stdint.h>
#include <assert.h>
#define RADIO_AIR_BIT_RATE_BPS 64000U
#define RADIO_AIR_SHARE_PERCENT 40U
#define RADIO_AIR_FRAME_OVERHEAD_BYTES 16U
#define RADIO_TX_COOLDOWN_MS 25U
#define RADIO_E22_MODE_SETTLE_MS 1000U
static uint32_t clock_ms, g_tx_quiet_until_ms, g_tx_air_budget_active;
static uint32_t g_e22_mode0_set_at_ms, g_aux_busy_count, aux_reads;
uint32_t radio_now_ms(void) { return clock_ms; }
uint32_t HAL_GetTick(void) { return clock_ms; }
uint8_t radio_e22_aux_high(void) { ++aux_reads; return 0; }
''' + source[ready_start:ready_end] + source[gates_start:gates_end] + r'''
int main(void) {
  clock_ms=100; g_e22_mode0_set_at_ms=0;
#if RFBOARD_LORA_SCHEDULER
  assert(!radio_e22_ready_for_uart());
  clock_ms=2000; assert(!radio_e22_ready_for_uart()); assert(aux_reads==1);
  uint32_t delay=radio_uart_air_ms(196)+RADIO_TX_COOLDOWN_MS;
  assert(delay>0); radio_uart_mark_tx_quiet(196); assert(radio_uart_air_busy());
  clock_ms+=delay; assert(!radio_uart_air_busy());
  clock_ms=UINT32_MAX-10U; radio_uart_mark_tx_quiet(196);
  assert(radio_uart_air_busy()); clock_ms+=delay; assert(!radio_uart_air_busy());
#else
  assert(radio_e22_ready_for_uart()); assert(!g_aux_busy_count && !aux_reads);
  assert(radio_uart_air_ms(196)==0);
  g_tx_quiet_until_ms=5000; g_tx_air_budget_active=1;
  assert(!radio_uart_air_busy());
  radio_uart_mark_tx_quiet(196); assert(!g_tx_quiet_until_ms);
  assert(!g_tx_air_budget_active && !radio_uart_air_busy());
  clock_ms=UINT32_MAX; assert(radio_e22_ready_for_uart());
  clock_ms=0; assert(radio_e22_ready_for_uart());
#endif
  (void)g_e22_mode0_set_at_ms; (void)g_aux_busy_count; (void)aux_reads;
}
'''
        compiler = shutil.which('clang') or shutil.which('cc')
        self.assertIsNotNone(compiler)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory); (path/'gates.c').write_text(program)
            for lora in (0,1):
                executable=path/f'gates-{lora}'
                subprocess.run([compiler,'-std=c11','-Wall','-Wextra','-Werror',
                    '-fsanitize=address,undefined',f'-DRFBOARD_LORA_SCHEDULER={lora}',
                    str(path/'gates.c'),'-o',str(executable)],check=True,capture_output=True)
                subprocess.run([str(executable)],check=True,capture_output=True)

if __name__ == '__main__':
    unittest.main()
