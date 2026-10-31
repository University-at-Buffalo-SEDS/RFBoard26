"""Exercise the actual UART parser after a proven CRC failure.

The native harness includes immutable transport vectors and adversarial framing
controls. Production parser code is extracted on every run, never copied into
the fixture, so a regression in radio.c cannot leave a stale test passing.
"""

import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class RadioRejectedSpanTests(unittest.TestCase):
    def test_rejected_span_recovery_and_parent_boundaries(self):
        source = (ROOT / "Core/Src/radio.c").read_text()
        start = source.index("/* Bounded resync state:")
        end = source.index("/* Push one RX chunk", start)
        template = (ROOT / "tests/native/radio_rejected_span.c").read_text()
        marker = "/* ACTUAL_PARSER_UNDER_TEST */"
        self.assertEqual(template.count(marker), 1)
        code = template.replace(marker, source[start:end])
        with tempfile.TemporaryDirectory() as directory:
            executable = str(Path(directory) / "radio_rejected_span")
            subprocess.run(
                [
                    "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                    "-I", str(ROOT / "Core/Inc"), "-I", str(ROOT / "tests"),
                    "-x", "c", "-", "-o", executable,
                ],
                input=code, text=True, check=True,
            )
            subprocess.run([executable], check=True)


if __name__ == "__main__":
    unittest.main()
