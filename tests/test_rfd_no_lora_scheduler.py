"""Run the production modem gates for RFD and explicit legacy LoRa builds."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

def function(source, signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]

class ModemSchedulingTests(unittest.TestCase):
    def test_rfd_capacity_is_bounded_without_lora_aux_or_turn_gates(self):
        source = (ROOT / 'Core/Src/radio.c').read_text()
        ready_start = source.index('static uint8_t radio_e22_ready_for_uart(void)\n{')
        ready_end = source.index('\n}\n', ready_start) + 3
        gates_start = source.index('static uint32_t radio_uart_air_ms(uint16_t len)\n{')
        gates_end = source.index('uint8_t radio_uart_tx_ready(void)', gates_start)
        support = """
#if !RFBOARD_LORA_SCHEDULER
#include <stddef.h>
#include <string.h>
#define RADIO_UART_FRAME_HEADER_SIZE 4U
#define RADIO_UART_FRAME_SYNC_0 0xa5
#define RADIO_UART_FRAME_SYNC_1 0x5a
#define RADIO_UART_PRIORITY_NETWORK_CONTROL 254U
typedef struct { uint16_t len; uint8_t priority; uint8_t data[1028]; } radio_tx_item_t;
static uint8_t g_tx_completed_service_credit_ms;
static uint64_t g_tx_closed_ack_ids[4];
static uint8_t g_tx_closed_ack_count, g_tx_closed_ack_cursor;
"""
        support += function(source, 'static uint8_t radio_read_uleb128(')
        support += function(source[source.index('/* Only canonical packets are independent'):], 'static uint8_t radio_uart_independent_control(')
        support += function((ROOT / 'tests/radio_crc_reference.h').read_text(), 'uint32_t launchcore_crc32(')
        support += "\n#endif\n"
        program = r'''
#include <stdint.h>
#include <assert.h>
#define RADIO_AIR_BIT_RATE_BPS 64000U
#define RADIO_BAUD_RATE 115200U
#define RADIO_PACKET_GAP_MS 55U
#define RADIO_PACKET_BURST_MS 100U
#define RADIO_AIR_SHARE_PERCENT 40U
#define RADIO_AIR_FRAME_OVERHEAD_BYTES 16U
#define RADIO_TX_COOLDOWN_MS 25U
#define RADIO_E22_MODE_SETTLE_MS 1000U
static uint32_t clock_ms, g_tx_quiet_until_ms, g_tx_air_budget_active;
#if !RFBOARD_LORA_SCHEDULER
static uint32_t g_tx_packet_started_ms, g_tx_packet_until_ms, g_tx_packet_active;
static uint32_t g_tx_last_forced_boundary_ms;
#endif
static uint32_t g_e22_mode0_set_at_ms, g_aux_busy_count, aux_reads;
uint32_t radio_now_ms(void) { return clock_ms; }
uint32_t HAL_GetTick(void) { return clock_ms; }
uint8_t radio_e22_aux_high(void) { ++aux_reads; return 0; }
''' + support + source[ready_start:ready_end] + source[gates_start:gates_end] + r'''
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
  assert(radio_uart_air_ms(196)==67);
  uint32_t delay=67; /*Airtime remains authoritative within a bounded batch.*/ radio_uart_mark_tx_quiet(196);
  assert(radio_uart_air_busy());
  clock_ms+=delay-1; assert(radio_uart_air_busy());
  ++clock_ms; assert(!radio_uart_air_busy());
  clock_ms=100000; radio_uart_mark_tx_quiet(196);
  assert(radio_uart_air_busy()); /* no idle credit */
  clock_ms+=delay; assert(!radio_uart_air_busy());
  clock_ms+=100; assert(!radio_uart_air_busy()); /* idle service retires the batch */
  clock_ms=UINT32_MAX-10U; radio_uart_mark_tx_quiet(196);
  assert(radio_uart_air_busy()); clock_ms+=delay; assert(!radio_uart_air_busy());
  g_tx_packet_active=0; g_tx_air_budget_active=0;
  clock_ms=1000;
  for (unsigned i=0;i<6;i++) {
    assert(!radio_uart_air_busy()); radio_uart_mark_tx_quiet(42);
    clock_ms+=4; radio_uart_mark_tx_drained(); clock_ms+=15;
  }
  /* Six short frames fit before 100ms, then the full serial gap is required. */
  assert(clock_ms==1114); assert(radio_uart_air_busy());
  clock_ms=1153; assert(radio_uart_air_busy());
  clock_ms=1154; assert(!radio_uart_air_busy());
  /* A late completion and timer wrap preserve the actual packet boundary. */
  g_tx_packet_active=1; g_tx_packet_started_ms=UINT32_MAX-200U;
  clock_ms=UINT32_MAX-20U; radio_uart_mark_tx_drained();
  clock_ms+=54; assert(radio_uart_air_busy());
  ++clock_ms; assert(!radio_uart_air_busy());
  clock_ms=UINT32_MAX; assert(radio_e22_ready_for_uart());
  clock_ms=0; assert(radio_e22_ready_for_uart());
  (void)radio_uart_close_packet_batch;
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
