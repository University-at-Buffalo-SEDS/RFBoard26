"""RF follows the station's network-time step policy while it leads from GPS.

A large GPS correction may step network time (a new SEDSnet epoch) only
outside Armed through Recovery: FLIGHT_STATE 6..16 inhibits steps, as the
station's absolute_time::may_step does. The telemetry service applies it to
the router whenever the cached flight state changes.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

MAIN = r'''
#include <assert.h>
#include "timesync_step_policy.h"
int main(void){
 assert(timesync_steps_allowed_for(false,0));
 assert(timesync_steps_allowed_for(false,7));
 for(unsigned s=0;s<=255;s++){
  bool inhibited=s>=6&&s<=16;
  assert(timesync_steps_allowed_for(true,(uint8_t)s)==!inhibited);
 }
 return 0;
}
'''


class TimesyncStepPolicy(unittest.TestCase):
    def test_policy_matches_the_station(self):
        with tempfile.TemporaryDirectory() as d:
            src = Path(d) / "t.c"
            src.write_text(MAIN)
            exe = Path(d) / "t"
            subprocess.run(["cc", "-std=c11", "-Wall", "-Werror", "-I", str(ROOT / "Core/Inc"), str(src), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_telemetry_applies_it_to_the_router(self):
        text = (ROOT / "Core/Src/telemetry.c").read_text()
        self.assertIn("timesync_steps_allowed_for(flight_state_cache_has_value()", text)
        self.assertIn("seds_router_set_timesync_steps_allowed(g_router.r, allowed)", text)


if __name__ == "__main__":
    unittest.main()
