"""SEDSnet receipts are never stale-dropped or evicted by the RF radio queue.

SEDSnet hands RF each frame with its own transport priority (ReliableAck,
ReliablePartialAck and ReliablePacketRequest 255, managed GET/value 254).
Only the legacy whole-frame path (radio_uart_enqueue_frame) classifies frames
itself, ages them out after RADIO_UART_TX_MAX_AGE_MS and evicts the lowest
priority; the router path refuses with HAL_BUSY instead, so SEDSnet keeps the
receipt. These guards keep the router path off the dropping one.
"""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


def _function(source: str, signature: str) -> str:
    start = source.index(signature)
    return source[start:source.index('\n}\n', start) + 3]


class RadioReceiptAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.radio = (ROOT / 'Core/Src/radio.c').read_text()

    def test_router_side_uses_sdk_priority_admission(self):
        telemetry = (ROOT / 'Core/Src/telemetry.c').read_text()
        send = _function(telemetry, 'static SedsResult radio_tx_send(')
        self.assertIn('radio_uart_send_bytes_priority(bytes, len, priority)', send)
        prio = _function(self.radio, 'HAL_StatusTypeDef radio_uart_send_bytes_priority(')
        self.assertIn('radio_uart_enqueue_sedsnet_parts(', prio)
        self.assertNotIn('radio_uart_enqueue_frame(', prio)

    def test_router_admission_never_ages_out_or_evicts(self):
        parts = _function(self.radio, 'static HAL_StatusTypeDef radio_uart_enqueue_sedsnet_parts(')
        for call in ('radio_uart_drop_stale_locked(', 'radio_uart_remove_queue_index(',
                     'radio_uart_classify_packet_drop('):
            self.assertNotIn(call, parts)
        # Receipts (priority 255) are independent controls: they get the
        # reserved slots and are selected ahead of 254 managed traffic.
        self.assertIn('const uint8_t control = priority >= RADIO_UART_PRIORITY_NETWORK_CONTROL', parts)

    def test_only_the_legacy_frame_path_ages_out_frames(self):
        self.assertEqual(self.radio.count('radio_uart_drop_stale_locked(radio_now_ms())'), 1)
        legacy = _function(self.radio, 'static HAL_StatusTypeDef radio_uart_enqueue_frame(')
        self.assertIn('radio_uart_drop_stale_locked(radio_now_ms())', legacy)


if __name__ == '__main__':
    unittest.main()
