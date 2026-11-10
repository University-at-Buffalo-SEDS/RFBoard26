"""RF's reliable retransmit interval is sized to the measured radio ACK round trip.

With carrier sense on the faithful RFD900x2 model the RF->GS reliable ACK round
trip measured p50 239 ms, p95 505 ms, p99 574 ms. A 250 ms interval re-sent
about half of all reliable frames before their ACK could return (battery
samples went on air about 1.8 times each). The total retry window stays about
4.5 s.
"""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class ReliableRetransmitSizingTests(unittest.TestCase):
    def test_interval_covers_the_ack_round_trip_and_window_is_kept(self):
        source = (ROOT / 'Core/Src/telemetry.c').read_text()
        interval = int(re.search(r'tuning\.reliable_retransmit_ms = (\d+)U;', source).group(1))
        retries = int(re.search(r'tuning\.reliable_max_retries = (\d+)U;', source).group(1))
        self.assertGreaterEqual(interval, 505, 'below the measured ACK round-trip p95')
        self.assertLessEqual(interval, 700)
        self.assertTrue(4000 <= interval * retries <= 5000, interval * retries)


if __name__ == '__main__':
    unittest.main()
