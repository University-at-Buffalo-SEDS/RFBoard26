"""A refused FLIGHT_STATE delivery is acknowledged and not applied (actual SDK).

The production cache callback runs inside the SDK's receive and relay. A
callback error there withholds every receipt and the onward relay, so the
station retransmits a stale or refused write forever. Deterministic refusals
now return success without applying; transient ones keep failing.
"""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from native_sdk import prepare_native_sdk

ROOT = Path(__file__).resolve().parents[1]
STUBS = {
    'main.h': '#pragma once\n#include <stdint.h>\nuint32_t HAL_GetTick(void);\n',
    'persistent_store.h': '#pragma once\n#include <stddef.h>\n#include <stdint.h>\n#include <stdbool.h>\n'
                          'typedef int launchcore_persist_status_t;\n#define LAUNCHCORE_PERSIST_OK 0\n'
                          '#define LAUNCHCORE_PERSIST_NOT_FOUND 1\n'
                          'launchcore_persist_status_t persistent_store_init(void);\n'
                          'launchcore_persist_status_t persistent_store_get(uint32_t,void*,size_t*);\n'
                          'launchcore_persist_status_t persistent_store_set(uint32_t,const void*,size_t);\n',
    'board_recovery_transport.h': '#pragma once\n#include <stdbool.h>\n#include "sedsnet.h"\n'
                                  '#include "sedsnet_config.h"\n'
                                  'void board_recovery_transport_set_router(SedsRouter*);\n'
                                  'uint8_t board_recovery_managed_update_allowed(const SedsPacketView*);\n'
                                  'uint8_t board_recovery_managed_report_ignorable(const SedsPacketView*);\n'
                                  'extern volatile uint32_t g_reliable_refusal_count;\n',
}


class ManagedRefusalReceiptTests(unittest.TestCase):
    def test_refused_flight_state_is_acknowledged_and_not_applied(self):
        sdk = prepare_native_sdk(ROOT)
        with tempfile.TemporaryDirectory(prefix='managed-refusal-receipt-') as tmp:
            path = Path(tmp)
            for name, text in STUBS.items():
                (path / name).write_text(text)
            (path / 'transport.c').write_text(
                '#include "board_recovery_transport.h"\n'
                'static SedsRouter *router;\n'
                'void board_recovery_transport_set_router(SedsRouter *r) { router = r; }\n'
                'uint8_t board_recovery_managed_report_ignorable(const SedsPacketView *p)\n'
                '{ (void)router; return p && p->sender_len == 2 && p->sender[0] != \'G\'; }\n'
                'volatile uint32_t g_reliable_refusal_count;\n')
            exe = path / 'refusal'
            command = ['cc', '-std=c11', '-D_XOPEN_SOURCE=700', '-DTELEMETRY_ENABLED=1', '-Wall', '-Wextra',
                       '-Werror', '-g', '-fsanitize=address,undefined', '-I' + str(path),
                       '-I' + str(ROOT / 'Core/Inc'), '-I' + str(Path(sdk['header']).parent),
                       str(ROOT / 'Core/Src/flight_state_cache.c'), str(path / 'transport.c'),
                       str(ROOT / 'tests/native/managed_refusal_receipt.in.c'), sdk['archive'],
                       '-lpthread', '-lm']
            if sys.platform.startswith('linux'):
                command.append('-ldl')
            subprocess.run(command + ['-o', str(exe)], check=True)
            result = subprocess.run([str(exe), str(ROOT / 'config/sedsnet.json')],
                                    capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        print(result.stdout.strip(), flush=True)
        self.assertIn('PASS refused and malformed FLIGHT_STATE acknowledged', result.stdout)


if __name__ == '__main__':
    unittest.main()
