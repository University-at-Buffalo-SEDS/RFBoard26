"""A wrongly anchored network clock is restarted from the network thread.

Bench evidence (Actuator Board): its SEDSnet clock read about an RF board's
uptime while the station's clock was UTC. A relayed consumer never completes a
verified time exchange, so only a timesync restart lets the next announce
anchor it again. RF runs the same check against its station clock. The actual
telemetry_update_network_health() runs with modeled SDK calls.
"""
from pathlib import Path
import subprocess,tempfile,unittest
from router_lifecycle_fixture import function
ROOT=Path(__file__).resolve().parents[1]
class NetworkClockResync(unittest.TestCase):
 def test_suspect_clock_restarts_timesync_and_counts_it(self):
  body=function((ROOT/'Core/Src/telemetry.c').read_text(),'telemetry_update_network_health')
  code=r'''#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef int SedsResult;typedef int SedsRouter;
#define SEDS_OK 0
#define SEDS_FIRMWARE_SIM_TEST 0
static SedsRouter router;static uint64_t net;static bool valid,suspect;static unsigned configures,checks;
static uint32_t g_telemetry_timesync_valid,g_telemetry_discovery_seen=1,g_radio_link_seen=1,g_telemetry_network_ready,g_network_clock_resyncs;
static SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){assert(r==&router);if(!valid)return -1;*out=net;return SEDS_OK;}
static bool board_recovery_network_clock_resync_needed(uint64_t n){assert(n==net);checks++;return suspect;}
static SedsResult telemetry_configure_timesync_locked(SedsRouter*r){assert(r==&router);configures++;return SEDS_OK;}
'''+body+r'''
int main(void){
 valid=false;telemetry_update_network_health(&router);assert(!configures&&!checks&&!g_telemetry_timesync_valid);
 valid=true;net=1794183600000ULL;telemetry_update_network_health(&router);assert(!configures&&checks==1&&g_telemetry_network_ready);
 net=5006972U;suspect=true;telemetry_update_network_health(&router);assert(configures==1&&g_network_clock_resyncs==1);
 suspect=false;telemetry_update_network_health(&router);assert(configures==1&&g_network_clock_resyncs==1);
 return 0;}
'''
  with tempfile.TemporaryDirectory() as d:
   exe=Path(d)/'resync';out=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=code,text=True,capture_output=True)
   self.assertEqual(out.returncode,0,out.stderr);subprocess.run([str(exe)],check=True,timeout=10)
if __name__=='__main__':unittest.main()
