"""Real SDK + live constructor/cache/RX with controlled ThreadX interleavings.

Hardware callbacks are inert. These host tests prove ownership and retry
contracts, not physical output, timing, RAM or deployment suitability.
"""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from native_sdk import prepare_native_sdk
from router_lifecycle_fixture import MOCK_HEADERS, assemble

ROOT = Path(__file__).resolve().parents[1]

class RouterLifecycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdk = prepare_native_sdk(ROOT)
        cls.tmp = tempfile.TemporaryDirectory(prefix='router-lifecycle-')
        cls.path = Path(cls.tmp.name)
        for name, contents in MOCK_HEADERS.items():
            (cls.path / name).write_text(contents)
        source = cls.path / 'lifecycle.c'
        source.write_text(assemble(ROOT))
        cls.exe = cls.path / 'lifecycle'
        command = ['cc', '-std=c11', '-D_XOPEN_SOURCE=700', '-DTELEMETRY_ENABLED=1',
                   '-Wall', '-Wextra', '-Werror', '-g', '-fsanitize=address,undefined',
                   '-DCACHE_SOURCE="' + str(ROOT / 'Core/Src/flight_state_cache.c') + '"',
                   '-I' + str(cls.path), '-I' + str(ROOT / 'Core/Inc'),
                   '-I' + str(Path(cls.sdk['header']).parent), str(source), cls.sdk['archive']]
        # C-call redirection in the fixture forwards to the real library;
        # GNU --wrap and Linux-only -ldl are not required on macOS.
        if sys.platform == 'win32' and cls.sdk['archive'].endswith('.lib'):
            raise RuntimeError('These native C sanitizer tests require a GNU-compatible cc and pthreads; use Linux/WSL or a matching GNU host SDK toolchain, not MSVC.')
        command += ['-lpthread', '-lm']
        if sys.platform.startswith('linux'):
            command.append('-ldl')
        command += ['-o', str(cls.exe)]
        subprocess.run(command, check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def case(self, name):
        result = subprocess.run([str(self.exe), str(ROOT / 'config/sedsnet.json'), name],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_two_initializers_and_actual_rx_wait_for_complete_registration(self): self.case('race')
    def test_isr_startup_missing_mutex_and_failed_acquisition_do_no_sdk_work(self): self.case('context')
    def test_packet_store_failure_is_private_and_retries_after_deadline(self): self.case('1')
    def test_timesync_configuration_failure_cannot_publish_freed_router(self): self.case('2')
    def test_ota_failure_cannot_publish_freed_router(self): self.case('3')
    def test_enable_failure_cannot_advertise_ready_recovery(self): self.case('4')
    def test_callback_failure_cannot_advertise_ready_recovery(self): self.case('5')
    def test_retry_deadline_survives_uint32_rollover(self): self.case('rollover')
    def test_zero_deadline_sentinel_has_at_most_one_extra_tick(self): self.case('zero_deadline')
    def test_unlock_failure_is_not_reported_as_success(self): self.case('leave')
    def test_unlock_failure_preserves_primary_registration_error(self): self.case('primary_error')

    def test_initial_timesync_poll_failure_precedes_cache_readiness(self): self.case('8')

if __name__ == '__main__':
    unittest.main()
