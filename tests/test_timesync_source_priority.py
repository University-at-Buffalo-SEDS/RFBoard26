"""Compile the actual board priority helpers independently of HAL/ThreadX."""
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class TimeSyncPriorityTests(unittest.TestCase):
    def test_holdover_yields_to_rf_and_groundstation_preserving_source_priority(self):
        source = (ROOT / "Core/Src/telemetry.c").read_text()
        functions = []
        for name in ("telemetry_timesync_is_source", "telemetry_timesync_priority"):
            match = re.search(r"static uint(?:8|64)_t " + name + r"\(void\)\s*\{[^}]*\}", source)
            self.assertIsNotNone(match, name)
            functions.append(match.group())
        self.assertRegex(source, r"seds_router_configure_timesync\(\s*router, true, telemetry_timesync_role\(\), telemetry_timesync_priority\(\)")
        with tempfile.TemporaryDirectory() as directory:
            for configured in (-1, 0, 1, 50, 123):
                with self.subTest(configured_source_priority=configured):
                    expected = 100 if configured < 0 else configured
                    code = '#include <stdint.h>\n#include <assert.h>\n'
                    code += f'#define TELEMETRY_TIMESYNC_MASTER_PRIO ({configured})\n'
                    code += 'static uint8_t g_local_unix_valid;\n'
                    code += '\n'.join(functions)
                    # Without GPS UTC even a configured source is a
                    # priority-100 consumer that yields to RF and the GS.
                    code += '\nint main(void) { assert(!telemetry_timesync_is_source());'
                    code += ' assert(telemetry_timesync_priority() == 100ULL);'
                    code += ' g_local_unix_valid = 1U;'
                    code += f' assert(telemetry_timesync_priority() == {expected}ULL);'
                    if configured < 0:
                        code += ' assert(telemetry_timesync_priority() > 1); assert(telemetry_timesync_priority() > 50);'
                    code += ' return 0; }\n'
                    binary = pathlib.Path(directory) / 'timesync-priority'
                    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-x', 'c', '-', '-o', str(binary)], input=code, text=True, check=True, capture_output=True)
                    subprocess.run([str(binary)], check=True)
