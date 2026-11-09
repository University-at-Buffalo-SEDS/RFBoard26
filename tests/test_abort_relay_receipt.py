"""The operator ABORT crosses the RF relay and its receipt returns (actual SDK).

The GS sends ABORT with seven endpoints and no target senders. RF and PB host
no handler for any of them, so the GS's receipt cohort is the FC alone; RF
must forward the ABORT to CAN and the FC's end-to-end receipt back over the
radio. A failing FC endpoint handler withholds that receipt, so the GS
retransmits for its whole retry budget and the FC re-runs its handlers per
copy: that is the FC handler bug, not an RF or PB relay defect.
"""
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest
from native_sdk import prepare_native_sdk

ROOT = Path(__file__).resolve().parents[1]


class AbortRelayReceiptTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sdk = prepare_native_sdk(ROOT)
        cls.tmp = tempfile.TemporaryDirectory(prefix='abort-relay-receipt-')
        exe = Path(cls.tmp.name) / 'relay'
        command = ['cc', '-std=c11', '-D_XOPEN_SOURCE=700', '-DTELEMETRY_ENABLED=1', '-Wall', '-Wextra',
                   '-Werror', '-g', '-fsanitize=address,undefined', '-I' + str(ROOT / 'Core/Inc'),
                   '-I' + str(Path(sdk['header']).parent),
                   str(ROOT / 'tests/native/abort_relay_receipt.in.c'), sdk['archive'], '-lpthread', '-lm']
        if sys.platform.startswith('linux'):
            command.append('-ldl')
        subprocess.run(command + ['-o', str(exe)], check=True)
        cls.exe = exe

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_case(self, mode):
        result = subprocess.run([str(self.exe), str(ROOT / 'config/sedsnet.json'), mode],
                                capture_output=True, text=True, timeout=120)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        print(result.stdout.strip(), flush=True)
        line = result.stdout.strip().splitlines()[-1]
        return {k: (int(v) if v.isdigit() else v) for k, v in re.findall(r'(\w+)=(\w+)', line)}, result.stdout

    def test_fc_receipt_returns_through_rf_and_ends_retransmission(self):
        stats, out = self.run_case('ok')
        self.assertIn('GS frozen ABORT receipt cohort=1', out)
        self.assertEqual(stats['pb_receipts'], 0)
        self.assertGreaterEqual(stats['fc_receipts'], 1)
        self.assertGreaterEqual(stats['gs_receipts'], 1)
        self.assertEqual(stats['gs_abort_pending'], 0)
        self.assertLessEqual(stats['gs_abort_tx'], 2)
        self.assertEqual(stats['fc_abort_calls'], 1)

    def test_failing_fc_endpoint_withholds_the_receipt(self):
        stats, _ = self.run_case('error')
        self.assertEqual(stats['gs_receipts'], 0)
        self.assertEqual(stats['fc_receipts'], 0)
        self.assertGreater(stats['gs_abort_tx'], 5)
        self.assertGreater(stats['fc_abort_calls'], 1)


if __name__ == '__main__':
    unittest.main()
