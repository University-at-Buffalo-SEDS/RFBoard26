"""RF is a SEDSnet time source only with GPS UTC (actual SDK, live functions).

A SEDSnet Source announces even without a date, so an RF board without a GPS
fix used to announce its uptime as network time at priority 1. A board that
anchored to that first (the Actuator Board behind the gateway) kept an
uptime-based clock: announces are only slewed at 5% and the verified exchange
does not cross relays. The production telemetry.c timesync functions run here
against the real SDK with a relayed consumer peer.
"""
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest
from native_sdk import prepare_native_sdk
from router_lifecycle_fixture import function

ROOT = Path(__file__).resolve().parents[1]


class TimesyncGpsGateTests(unittest.TestCase):
    def test_no_fix_never_announces_and_fix_leads_with_gps_utc(self):
        sdk = prepare_native_sdk(ROOT)
        source = (ROOT / 'Core/Src/telemetry.c').read_text()
        program = (ROOT / 'tests/native/timesync_gps_gate.in.c').read_text()
        holdover = re.search(r'(?m)^#define TELEMETRY_TIMESYNC_GPS_HOLDOVER_MS\s+(\S+)', source)
        self.assertIsNotNone(holdover)
        program = program.replace('@@HOLDOVER@@',
                                  '#define TELEMETRY_TIMESYNC_GPS_HOLDOVER_MS ' + holdover[1])
        for name in re.findall(r'@@TELEMETRY:([A-Za-z_0-9]+)@@', program):
            program = program.replace('@@TELEMETRY:' + name + '@@', function(source, name))
        self.assertNotIn('@@', program)
        with tempfile.TemporaryDirectory(prefix='timesync-gps-gate-') as directory:
            path = Path(directory)
            (path / 'gate.c').write_text(program)
            exe = path / 'gate'
            command = ['cc', '-std=c11', '-D_XOPEN_SOURCE=700', '-Wall', '-Wextra', '-Werror',
                       '-g', '-fsanitize=address,undefined',
                       '-I' + str(Path(sdk['header']).parent), str(path / 'gate.c'),
                       sdk['archive'], '-lpthread', '-lm']
            if sys.platform.startswith('linux'):
                command.append('-ldl')
            subprocess.run(command + ['-o', str(exe)], check=True)
            result = subprocess.run([str(exe), str(ROOT / 'config/sedsnet.json')],
                                    capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        print(result.stdout.strip(), flush=True)
        self.assertIn('NO_FIX announces=0 peer_time=none', result.stdout)
        self.assertIn('HOLDOVER announces_after_expiry=0', result.stdout)


if __name__ == '__main__':
    unittest.main()
