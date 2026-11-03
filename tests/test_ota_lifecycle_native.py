"""Actual OTA/cache/query ownership; fake SDK/HAL/LC/storage boundaries.
No physical flash, image verification, original-UTC or packed-transport claim.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
CASES=['healthy_idle', 'manual_reset', 'finish_launch', 'restored_launch', 'unsafe_phase', 'unknown_identity', 'exhausted_identity', 'wrong_principal', 'malformed', 'wrong_tuple', 'wrong_address', 'wrong_peer_stream', 'reentrant_begin', 'reentrant_write', 'active_abort', 'bad_offset', 'reentrant_finish', 'launch_idle', 'stop', 'corrupt_health', 'corrupt_floor', 'epoch_changed', 'identity_changed', 'graceful_close', 'stream_reset', 'inactive_abort', 'new_accept', 'clock_rollback', 'reentrant_confirm', 'failed_leave', 'verified_recovery', 'no_clear_by_apply', 'busy_reset', 'closed_reset', 'unhealthy_reset', 'unhealthy_abort', 'unhealthy_close', 'busy_update_reset', 'wrong_cancel_tuple', 'same_session_begin', 'same_session_begin_failure']
class OtaLifecycleTests(unittest.TestCase):
 def test_actual_owner_lifecycle(self):
  fixture=ROOT/'tests/fixtures/ota_lifecycle'
  text=(ROOT/'Core/Src/board_recovery_transport.c').read_text()
  authority=text[text.index('static SedsRouter *recovery_router;'):text.index('static SedsResult commissioning_status')]
  with tempfile.TemporaryDirectory(prefix='ota-lifecycle-') as tmp:
   p=Path(tmp);a=p/'authority.c';a.write_text('#include "board_recovery_transport.h"\n#include "board_recovery_floor.h"\n#include "persistent_store.h"\n'+authority)
   cmd=['cc','-std=c11','-DTELEMETRY_ENABLED','-Wall','-Wextra','-Werror','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-pie','-no-pie']
   if ROOT.name=='DAQ-Board':cmd+=['-DDATA4_DAQ']
   cmd+=['-I'+str(fixture),'-I'+str(ROOT/'Core/Inc'),'-I'+str(ROOT/'Bootloader'),*[str(ROOT/'Core/Src'/n) for n in ['ota_stream.c','flight_state_cache.c','board_recovery_query.c']],str(a),str(fixture/'harness.c'),'-o',str(p/'test')]
   subprocess.run(cmd,check=True)
   for case in CASES:
    with self.subTest(case=case):subprocess.run([str(p/'test'),case],check=True,timeout=10)
