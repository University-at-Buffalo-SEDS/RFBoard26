"""Telemetry-thread stack budget for the router's side-TX callback chain.

Under full-bay traffic the router reaches the radio TX callback about 8.5 KiB
deep in the 14 KiB telemetry stack. Anything the callback or its encoding step
adds there is paid on the thread's deepest path, so it must stay small.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
# Bytes the radio TX callback (radio_uart_send_bytes_priority and the queue
# admission it calls) may add to the router's deepest call chain.
RADIO_TX_CALLBACK_BUDGET = 192


def _function(source: str, signature: str) -> str:
    start = source.index(signature)
    return source[start:source.index('\n}\n', start) + 3]


class TelemetryStackBudgetTests(unittest.TestCase):
    def test_router_never_attempts_zstd_compression_on_transmit(self):
        """ZSTD_compress needs a 3.8 KiB stack context plus a ~30 KiB heap
        workspace; RF's newlib heap is under 2 KiB, so the attempt can only
        fail after spending the stack at the bottom of the router chain."""
        source = (ROOT / 'Core/Src/telemetry.c').read_text()
        init = _function(source, 'static SedsResult init_telemetry_router_locked(void) {')
        get = init.index('seds_get_runtime_tuning_config(&tuning)')
        set_ = init.index('seds_set_runtime_tuning_config(&tuning)')
        threshold = re.search(r'tuning\.payload_compress_threshold\s*=\s*SIZE_MAX\s*;', init)
        self.assertIsNotNone(threshold, 'RF must disable transmit-side payload compression')
        self.assertTrue(get < threshold.start() < set_)

    @unittest.skipUnless(shutil.which('arm-none-eabi-gcc'), 'ARM GNU tools required')
    def test_radio_tx_callback_frame_fits_budget(self):
        source = (ROOT / 'Core/Src/radio.c').read_text()
        types = '\n'.join(re.search(r'typedef struct \{[^}]*\} ' + name + ';', source).group()
                          for name in ['radio_tx_item_t', 'radio_tx_queue_item_t'])
        send = _function(source, 'HAL_StatusTypeDef radio_uart_send_bytes_priority(')
        # The admission helper(s) between the queue lock and send.
        start = source.index('/* Accepted frames retain owned storage until DMA.')
        admission = source[start:source.index('/* Only canonical packets', start)]
        unit = r'''
#include <stddef.h>
#include <stdint.h>
#include <string.h>
typedef unsigned HAL_StatusTypeDef;
#define HAL_OK 0U
#define HAL_ERROR 1U
#define HAL_BUSY 2U
#define RFBOARD_LORA_SCHEDULER 0
#define RFBOARD_RADIO_LISTEN_ONLY 0
#define RADIO_UART_FRAME_HEADER_SIZE 4U
#define RADIO_UART_MAX_PAYLOAD_SIZE 1024U
#define RADIO_UART_FRAME_BUF_SIZE 1028U
#define RADIO_UART_FRAME_SYNC_0 0xa5U
#define RADIO_UART_FRAME_SYNC_1 0x5aU
#define RADIO_UART_TX_QUEUE_SLOTS 12U
#define RADIO_UART_TX_QUEUE_BYTE_CAPACITY 2056U
#define RADIO_UART_TX_CONTROL_RESERVE_BYTES 256U
#define RADIO_UART_TX_CONTROL_RESERVE_SLOTS 2U
#define RADIO_UART_TX_NORMAL_LOOKAHEAD_BYTES 196U
#define RADIO_UART_PRIORITY_NETWORK_CONTROL 254U
#define RADIO_UART_SCHED_FALLBACK_FLOW 0U
#define RADIO_UART_CLASS_NETWORK_CONTROL 2U
#define RADIO_UART_CLASS_APPLICATION 0U
''' + types + r'''
extern void *g_huart;
extern radio_tx_queue_item_t g_tx_queue[RADIO_UART_TX_QUEUE_SLOTS];
extern radio_tx_item_t g_tx_dma_item;
extern uint8_t g_tx_dma_busy;
extern uint8_t g_tx_queue_bytes[RADIO_UART_TX_QUEUE_BYTE_CAPACITY];
extern uint32_t g_tx_count, g_tx_queue_bytes_used, g_tx_busy, g_tx_enqueued, g_tx_startup_drops;
HAL_StatusTypeDef radio_uart_lock_tx_queue(void);
void radio_uart_unlock_tx_queue(void);
uint8_t radio_uart_independent_control(const uint8_t *data, uint16_t len, uint8_t priority);
uint32_t launchcore_crc32(const void *data, size_t len);
uint32_t radio_uart_owned_frame_count(void);
uint32_t radio_now_ms(void);
void radio_uart_note_queue_depth_locked(void);
uint8_t radio_uart_tx_ready(void);
uint8_t radio_e22_ready_for_uart(void);
''' + admission + send
        with tempfile.TemporaryDirectory() as tmp:
            obj = Path(tmp) / 'radio_tx.o'
            subprocess.run(['arm-none-eabi-gcc', '-mthumb', '-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16',
                            '-mfloat-abi=hard', '-Os', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                            '-Wno-unused-function', '-fstack-usage', '-x', 'c', '-', '-c',
                            '-o', str(obj)], input=unit, text=True, check=True,
                           capture_output=True)
            usage = {}
            for line in obj.with_suffix('.su').read_text().splitlines():
                name, size, _kind = line.split('\t')
                usage[name.rsplit(':', 1)[-1]] = int(size)
        self.assertIn('radio_uart_send_bytes_priority', usage)
        total = sum(usage.values())
        self.assertLessEqual(total, RADIO_TX_CALLBACK_BUDGET,
                             f'radio TX callback frames {usage} exceed the stack budget')


if __name__ == '__main__':
    unittest.main()
