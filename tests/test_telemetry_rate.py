import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
PREFIX = "rf"

class TelemetryRateTests(unittest.TestCase):
    def compile_rate(self, rate, expected, valid=True):
        with tempfile.TemporaryDirectory() as directory:
            binary = pathlib.Path(directory) / "rate-test"
            checks = ""
            if PREFIX == "fc":
                checks = """
  assert(fc_telemetry_rate_allow(102, 0));
  assert(!fc_telemetry_rate_allow(102, 1));
  assert(fc_telemetry_rate_allow(103, 1));
  assert(fc_telemetry_rate_allow(102, EXPECTED));
  assert(fc_telemetry_rate_allow(1, 0));
  assert(fc_telemetry_rate_allow(1, 0));
  assert(fc_telemetry_rate_allow(128, UINT32_MAX - 10));
  assert(!fc_telemetry_rate_allow(128, 0));
  assert(fc_telemetry_rate_allow(128, EXPECTED));
""".replace("EXPECTED", str(expected))
            code = '#include "telemetry_rate.h"\n#include <assert.h>\nint main(void) {\n'
            code += f'assert({PREFIX}_telemetry_period_ms() == {expected});\n' + checks + '}\n'
            cmd = ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "Core/Inc")]
            if rate is not None:
                cmd += [f"-D{PREFIX.upper()}_TELEMETRY_RATE_HZ={rate}"]
            cmd += [str(ROOT / "Core/Src/telemetry_rate.c"), "-x", "c", "-", "-o", str(binary)]
            result = subprocess.run(cmd, input=code, text=True, capture_output=True)
            if valid:
                self.assertEqual(result.returncode, 0, result.stderr)
                subprocess.run([str(binary)], check=True)
            else:
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("must be a whole number", result.stderr)

    def test_default_one_hz(self):
        self.compile_rate(None, 1000)

    def test_board_override_four_hz(self):
        self.compile_rate(4, 250)

    def test_invalid_rates_rejected(self):
        for rate in (0, 1001):
            with self.subTest(rate=rate):
                self.compile_rate(rate, 1000, valid=False)
