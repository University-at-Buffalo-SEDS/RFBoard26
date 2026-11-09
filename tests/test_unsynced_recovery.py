"""Recovery and command freshness without a synced SEDSnet network clock.

Bench evidence (Actuator Board): its network clock was unsynced or anchored
to an RF board's uptime, so every GS QUERY looked stale and RECOVER was always
refused. RF now uses max(valid network time, station clock), where the station
clock is the newest GS QUERY timestamp advanced by the board tick. The actual
recovery transport, commissioning, storage, boot identity and floor C run over
the recovery_wire flash fixture; only the SDK clock and the tick are replaced.
"""
from pathlib import Path
import os,subprocess,tempfile,unittest
from native_sdk import configured_dependency_source
ROOT=Path(__file__).resolve().parents[1]
CLOCK_OLD='SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){sdk_unlocked();assert(r);*out=clock_ms;return SEDS_OK;}'
CLOCK_NEW=r'''static int net_mode; /* 0 synced (clock_ms), 1 unavailable, 2 valid but uptime-like */
SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){sdk_unlocked();assert(r);
 if(net_mode==1)return SEDS_IO;
 *out=net_mode==2?5006972U:clock_ms;return SEDS_OK;}'''
TICK_OLD='ULONG tx_time_get(void){return 1;}'
TICK_NEW='static ULONG ticks=1;ULONG tx_time_get(void){return ticks;}'
MAIN=r'''
#define T0 UINT64_C(1794183589988)
static uint8_t body[32];static SedsPacketView packet;
static void query(SedsRouter*r,uint64_t nonce,uint64_t time){
 memset(body,0,sizeof body);body[0]=1;body[1]=BOARD_RECOVERY_QUERY;put64(body+4,nonce);put64(body+24,77);
 packet.timestamp=time;request(r,&packet,false);
}
static uint8_t recover(SedsRouter*r,uint64_t nonce,uint64_t generation,uint64_t time){
 memset(body,0,sizeof body);body[0]=1;body[1]=BOARD_RECOVERY_RECOVER;put64(body+4,nonce);put64(body+12,generation);body[20]=1;put64(body+24,77);
 packet.timestamp=time;request(r,&packet,true);return response[1];
}
int main(int argc,char**argv){
 assert(argc==2);(void)send;(void)response_epoch;(void)get64;
 memset(disk,255,sizeof disk);launchcore_storage_set_driver(&launchcore_board_storage_driver);uint64_t prior=41;uint8_t original=1;uint64_t floor[2]={77,100};
 assert(launchcore_persist_init()==0);assert(launchcore_persist_set(0x42524745U,&prior,8)==0);assert(launchcore_persist_set(0x46535445U,&original,1)==0);assert(launchcore_persist_set(0x4252464cU,floor,16)==0);
 fixture=false;(void)persistent_store_boot_begin();uint64_t generation=board_recovery_boot_generation();(void)board_recovery_floor_prime_boot();board_commissioning_probe_boot();flight_state_cache_restore();
 assert(persistent_store_runtime_start()==0);runtime=true;assert(persistent_store_controls_ready());
 int cookie=0;SedsRouter*r=(SedsRouter*)&cookie;assert(flight_state_cache_init(r)==0);board_recovery_transport_set_router(r);
 static uint32_t endpoint=SEDS_EP_BOARD_RECOVERY;
 packet=(SedsPacketView){.ty=200,.sender="GS",.sender_len=2,.endpoints=&endpoint,.num_endpoints=1,.payload=body,.payload_len=32};
 net_mode=!strcmp(argv[1],"uptime")?2:1;
 unsigned writes=media_writes;
 /* No GS time yet: nothing is fresh and nothing is written. */
 assert(board_recovery_station_now(r)==(net_mode==2?5006972U:0U));
 assert(recover(r,2,generation,T0)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
 if(!strcmp(argv[1],"stale")){
  /* The board tick advances the station clock without new traffic: a request
   * that is 16 s old by the GS QUERY's clock is refused. */
  query(r,3,T0+1);assert(response[1]==0);ticks+=16000U;
  assert(recover(r,4,generation,T0+2)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
  /* An older QUERY never moves the station clock back. */
  query(r,5,T0+3);assert(recover(r,6,generation,T0+4)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
  puts("station clock ages requests and never moves back PASS");return 0;
 }
 /* The GS QUERY's own time makes the station clock; RECOVER completes. */
 query(r,3,T0+1);assert(response[1]==0);
 assert(board_recovery_station_now(r)==T0+1);
 assert(recover(r,4,generation,T0+2)==BOARD_RECOVERY_RESULT_OK);
 assert(persistent_store_controls_ready()&&!persistent_store_commission_pending());
 /* The replay floor is active; managed/command writes use the same clock. */
 uint8_t state=0;SedsPacketView command={.ty=SEDS_DT_FLIGHT_STATE,.sender="GS",.sender_len=2,.payload=&state,.payload_len=1,.timestamp=T0+3};
 assert(board_recovery_floor_active()&&!board_recovery_command_is_stale(&command));
 command.timestamp=T0+3+2000U;assert(board_recovery_command_is_stale(&command));
 ticks+=16000U;command.timestamp=T0+3;assert(board_recovery_command_is_stale(&command));
 /* A wrongly anchored network clock is flagged once for a timesync restart;
  * the station clock then restarts from the next GS QUERY. */
 assert(!board_recovery_network_clock_resync_needed(T0+1+16000U+59000U));
 assert(board_recovery_network_clock_resync_needed(5006972U));
 assert(!board_recovery_network_clock_resync_needed(5006972U));
 query(r,5,T0+40000U);assert(!board_recovery_network_clock_resync_needed(T0+40000U));
 assert(board_recovery_station_now(r)==T0+40000U);
 puts("recovery and command freshness from the station clock without network time PASS");return 0;
}
'''
class UnsyncedRecovery(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.tmp=tempfile.TemporaryDirectory(prefix='unsynced-recovery-');p=Path(cls.tmp.name)
  sdk=Path(os.environ['SEDSNET_TEST_SOURCE']);lc=configured_dependency_source(ROOT,'sedslaunchcore','SDK_FREE_LAUNCHCORE_SOURCE')
  fixture=ROOT/'tests/fixtures/recovery_wire'
  harness=(fixture/'harness.c').read_text().split('int main(int argc,char**argv){',1)[0]
  assert CLOCK_OLD in harness and TICK_OLD in harness
  (p/'unsynced.c').write_text(harness.replace(CLOCK_OLD,CLOCK_NEW).replace(TICK_OLD,TICK_NEW)+MAIN)
  names=['board_recovery_wire.c','board_recovery_boot.c','board_recovery_transport.c','flight_state_cache.c','persistent_store.c','board_recovery_query.c','board_recovery_floor.c','board_commissioning.c','ota_stream.c']
  cls.exe=p/'unsynced'
  subprocess.run(['cc','-DEXPECTED_CAP=32','-DTELEMETRY_ENABLED=1','-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-function','-fsanitize=address,undefined','-I'+str(fixture),'-I'+str(ROOT/'Core/Inc'),'-I'+str(ROOT/'Bootloader'),'-I'+str(sdk/'C-Headers'),'-I'+str(lc/'bootloader/include'),'-I'+str(lc/'update_lib/include'),*[str(ROOT/'Core/Src'/n)for n in names],str(ROOT/'Bootloader/storage_dispatch.c'),str(lc/'bootloader/src/persist.c'),str(lc/'bootloader/src/crc32.c'),str(p/'unsynced.c'),'-o',str(cls.exe)],check=True)
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def run_case(self,name):
  out=subprocess.run([str(self.exe),name],capture_output=True,text=True,timeout=60)
  self.assertEqual(out.returncode,0,out.stdout+out.stderr)
 def test_recovery_without_network_time_uses_station_clock(self):self.run_case('unsynced')
 def test_recovery_with_uptime_based_network_clock(self):self.run_case('uptime')
 def test_station_clock_ages_requests_and_never_moves_back(self):self.run_case('stale')
if __name__=='__main__':unittest.main()
