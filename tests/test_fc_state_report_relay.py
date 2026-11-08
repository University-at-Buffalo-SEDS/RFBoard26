"""The RF relay must carry FC flight-state reports to the GS (no hardware).

FLIGHT_STATE is a managed network variable that only the primary station may
write on a board. The SDK runs each board's update callback inside its own
transmit and inside every relay; failing the FC's report there aborted the
FC transmit and the RF relay, so the GS never saw an FC flight state. A report
that is not a station write is ignorable: not applied, but never a failure.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(text, name):
    i = text.index(name + '(')
    start = text.rfind('\n', 0, i) + 1
    j = text.index('{', i)
    depth, k = 1, j + 1
    while depth:
        depth += (text[k] == '{') - (text[k] == '}')
        k += 1
    return text[start:k]


class FcStateReportRelayTests(unittest.TestCase):
    def test_only_non_station_flight_state_reports_are_ignorable(self):
        text = (ROOT / 'Core/Src/board_recovery_transport.c').read_text()
        code = r'''#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
typedef int SedsRouter;
typedef struct { uint32_t ty; const char *sender; size_t sender_len; } SedsPacketView;
#define SEDS_DT_FLIGHT_STATE 11U
static int router; static SedsRouter *recovery_router = &router;
bool seds_router_sender_matches_hostname(SedsRouter *r, const char *s, size_t len, const char *n, size_t nl)
{ assert(r); return s && len == nl && !memcmp(s, n, nl); }
''' + function(text, 'authorized_primary') + '\n' + function(text, 'board_recovery_managed_report_ignorable') + r'''
int main(void) {
  SedsPacketView p = { SEDS_DT_FLIGHT_STATE, "FC", 2 };
  assert(board_recovery_managed_report_ignorable(&p));
  p.sender = "RF"; assert(board_recovery_managed_report_ignorable(&p));
  p.sender = "GS"; assert(!board_recovery_managed_report_ignorable(&p));
  p.sender = NULL; p.sender_len = 0; assert(!board_recovery_managed_report_ignorable(&p));
  p.sender = "FC"; p.sender_len = 2; p.ty = 12U; assert(!board_recovery_managed_report_ignorable(&p));
  p.ty = SEDS_DT_FLIGHT_STATE; recovery_router = NULL; assert(!board_recovery_managed_report_ignorable(&p));
  assert(!board_recovery_managed_report_ignorable(NULL));
  return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / 'probe.c'
            exe = Path(tmp) / 'probe'
            src.write_text(code)
            subprocess.run(['cc', '-std=c17', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                            str(src), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_flight_state_callback_ignores_reports_before_failing(self):
        text = (ROOT / 'Core/Src/flight_state_cache.c').read_text()
        start = text.index('static SedsResult persist_update(const SedsPacketView *packet, void *user)\n{')
        body = function(text[start:], 'persist_update')
        denied = body.index('if (!board_recovery_managed_update_allowed(packet))')
        ignored = body.index('board_recovery_managed_report_ignorable(packet)', denied)
        self.assertLess(ignored, body.index('return SEDS_HANDLER_ERROR;', denied))
        self.assertIn('return SEDS_OK;', body[ignored:body.index('return SEDS_HANDLER_ERROR;', denied)])
        header = (ROOT / 'Core/Inc/board_recovery_transport.h').read_text()
        self.assertIn('board_recovery_managed_report_ignorable(const SedsPacketView *packet);', header)


if __name__ == '__main__':
    unittest.main()
