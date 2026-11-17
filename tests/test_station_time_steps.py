"""Station requests across station clock steps (RF Board, SEDSnet 6).

Bench (2026-11-16): the GS network clock stepped about 3 h 49 min back. Every
station request was judged by its raw timestamp against the board's clock and
the persisted recovery floor (the last Reset's station stamp), so FLIGHT_STATE
and recovery requests were refused for hours.

Every station stamp now carries its time-base epoch. The actual recovery
transport, commissioning, storage, boot identity and floor C run over the
recovery_wire flash fixture; only the SDK's network-stamp API is modelled
(tests/fixtures/recovery_wire/harness.c: one remembered step, a newer epoch
first heard newer_age ms ago, no network time).
"""
from pathlib import Path
import os, subprocess, tempfile, unittest
from native_sdk import configured_dependency_source

ROOT = Path(__file__).resolve().parents[1]
CLOCK_OLD = 'SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){sdk_unlocked();assert(r);*out=clock_ms;return SEDS_OK;}'
CLOCK_NEW = r'''static bool net_ok=true;
SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){sdk_unlocked();assert(r);
 if(!net_ok)return SEDS_IO;
 *out=clock_ms;return SEDS_OK;}'''
MAIN = r'''
#define T0 UINT64_C(1794183589988)
#define H4 INT64_C(14400000)
static uint8_t body[32];static SedsPacketView packet;
static uint8_t query(SedsRouter*r,uint64_t nonce,uint16_t te,uint64_t time){
 memset(body,0,sizeof body);body[0]=1;body[1]=BOARD_RECOVERY_QUERY;put64(body+4,nonce);put64(body+24,77);
 packet.timestamp=time;packet.time_epoch=te;request(r,&packet,false);return response[1];
}
static uint8_t recover(SedsRouter*r,uint64_t nonce,uint64_t generation,uint32_t epoch,uint16_t te,uint64_t time){
 memset(body,0,sizeof body);body[0]=1;body[1]=BOARD_RECOVERY_RECOVER;put64(body+4,nonce);put64(body+12,generation);
 for(unsigned i=0;i<4;i++)body[20+i]=(uint8_t)(epoch>>(8*i));
 put64(body+24,77);packet.timestamp=time;packet.time_epoch=te;request(r,&packet,true);return response[1];
}
/* A station FLIGHT_STATE as the board admits it while the floor is active. */
static bool admitted(uint16_t te,uint64_t time){
 uint8_t state=0;SedsPacketView c={.ty=SEDS_DT_FLIGHT_STATE,.sender="GS",.sender_len=2,.payload=&state,.payload_len=1,.timestamp=time,.time_epoch=te};
 return !board_recovery_command_is_stale(&c);
}
/* Reads the floor record directly (fixture mode: outside the storage owner). */
static uint64_t stored_floor(void){uint8_t w[16];size_t n=16;fixture=true;assert(launchcore_persist_get(0x4252464cU,w,&n)==0&&n==16);fixture=false;return get64(w+8);}
static void step(int64_t delta){prev_epoch=clock_epoch;clock_epoch++;step_delta=delta;clock_ms=(uint64_t)((int64_t)clock_ms+delta);}
int main(int argc,char**argv){
 assert(argc==2);const char*scenario=argv[1];(void)send;(void)response_epoch;
 memset(disk,255,sizeof disk);launchcore_storage_set_driver(&launchcore_board_storage_driver);uint64_t prior=41;uint8_t original=1;uint64_t floor[2]={77,100};
 assert(launchcore_persist_init()==0);assert(launchcore_persist_set(0x42524745U,&prior,8)==0);assert(launchcore_persist_set(0x46535445U,&original,1)==0);assert(launchcore_persist_set(0x4252464cU,floor,16)==0);
 fixture=false;(void)persistent_store_boot_begin();uint64_t generation=board_recovery_boot_generation();(void)board_recovery_floor_prime_boot();board_commissioning_probe_boot();flight_state_cache_restore();
 assert(persistent_store_runtime_start()==0);runtime=true;assert(persistent_store_controls_ready());
 int cookie=0;SedsRouter*r=(SedsRouter*)&cookie;assert(flight_state_cache_init(r)==0);board_recovery_transport_set_router(r);
 static uint32_t endpoint=SEDS_EP_BOARD_RECOVERY;
 packet=(SedsPacketView){.ty=200,.sender="GS",.sender_len=2,.endpoints=&endpoint,.num_endpoints=1,.payload=body,.payload_len=32};
 /* The first Reset at T0 (epoch 0). The floor record written before
  * SEDSnet 6 ({77,100}) reads as epoch 0. */
 clock_ms=T0;assert(query(r,3,0,T0)==0);assert(recover(r,4,generation,1,0,T0+1)==BOARD_RECOVERY_RESULT_OK);
 assert(persistent_store_controls_ready()&&!persistent_store_commission_pending());
 assert(board_recovery_floor_active()&&stored_floor()==network_stamp_key(0,T0+1));
 clock_ms=T0+2000;assert(admitted(0,T0+1990));assert(!admitted(0,T0+1));
 const uint64_t before=clock_ms;
 if(!strcmp(scenario,"step_back")||!strcmp(scenario,"step_forward")){
  step(!strcmp(scenario,"step_back")?-H4:INT64_C(13680000)); /* 4 h back or 3.8 h forward */
  const uint64_t now=clock_ms;
  /* The station's next request, on the new clock, is admitted at once. */
  assert(admitted(1,now));assert(admitted(1,now-14000));assert(!admitted(1,now-16000));assert(!admitted(1,now+2000));
  /* A late copy stamped before the step is judged by its age across it. */
  assert(admitted(0,before-1000));assert(!admitted(0,before-20000));
  /* A replay stamped before the Reset stays below the floor. */
  assert(!admitted(0,T0+1));assert(!admitted(0,T0));
  /* A stamp from an epoch this board cannot translate is stale. */
  clock_epoch=2;prev_epoch=1;assert(!admitted(0,before-1000));clock_epoch=1;prev_epoch=0;
  /* Reset after the step: newer than the floor, whatever its raw stamp. */
  const uint32_t e=response_epoch();
  assert(query(r,21,1,now+10)==0);assert(recover(r,22,generation,e,1,now+11)==BOARD_RECOVERY_RESULT_OK);
  assert(stored_floor()==network_stamp_key(1,now+11));
  clock_ms=now+20;
  /* Every request stamped before that Reset, in either epoch, is refused. */
  assert(!admitted(1,now+5));assert(!admitted(0,before-1000));assert(admitted(1,now+15));
  unsigned writes=media_writes;
  assert(recover(r,31,generation,response_epoch(),1,now+5)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
  assert(recover(r,4,generation,1,0,T0+1)!=BOARD_RECOVERY_RESULT_OK&&media_writes==writes);
  printf("%s: requests admitted across the step, Reset accepted, replays refused PASS\n",scenario);return 0;
 }
 if(!strcmp(scenario,"reanchor")){
  /* The GS stepped back (epoch 1); this board has heard of epoch 1 but not
   * re-anchored yet: fresh for at most the 15 s window. */
  const uint64_t gs_now=clock_ms-(uint64_t)H4;
  newer_age=2000;assert(admitted(1,gs_now));
  const uint32_t e=response_epoch();
  assert(query(r,41,1,gs_now)==0);assert(recover(r,42,generation,e,1,gs_now+1)==BOARD_RECOVERY_RESULT_OK);
  newer_age=16000;assert(!admitted(1,gs_now+50));
  unsigned writes=media_writes;
  assert(recover(r,51,generation,response_epoch(),1,gs_now+60)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
  puts("newer-epoch request admitted while re-anchoring, refused once overdue PASS");return 0;
 }
 if(!strcmp(scenario,"no_time")){
  /* No network time: nothing is fresh, nothing is written. */
  net_ok=false;assert(!admitted(0,T0+2000));unsigned writes=media_writes;
  assert(recover(r,61,generation,response_epoch(),0,T0+2001)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
  net_ok=true;assert(admitted(0,T0+2000));
  puts("no network time: station requests refused PASS");return 0;
 }
 return 2;
}
'''


class StationTimeSteps(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix='station-time-steps-')
        p = Path(cls.tmp.name)
        sdk = Path(os.environ['SEDSNET_TEST_SOURCE'])
        lc = configured_dependency_source(ROOT, 'sedslaunchcore', 'SDK_FREE_LAUNCHCORE_SOURCE')
        fixture = ROOT / 'tests/fixtures/recovery_wire'
        harness = (fixture / 'harness.c').read_text().split('int main(int argc,char**argv){', 1)[0]
        assert CLOCK_OLD in harness
        (p / 'steps.c').write_text(harness.replace(CLOCK_OLD, CLOCK_NEW) + MAIN)
        names = ['board_recovery_wire.c', 'board_recovery_boot.c', 'board_recovery_transport.c', 'flight_state_cache.c',
                 'persistent_store.c', 'board_recovery_query.c', 'board_recovery_floor.c', 'board_commissioning.c',
                 'ota_stream.c']
        cls.exe = p / 'steps'
        subprocess.run(['cc', '-DEXPECTED_CAP=32', '-DTELEMETRY_ENABLED=1', '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-Wno-unused-function', '-fsanitize=address,undefined', '-I' + str(fixture),
                        '-I' + str(ROOT / 'Core/Inc'), '-I' + str(ROOT / 'Bootloader'), '-I' + str(sdk / 'C-Headers'),
                        '-I' + str(lc / 'bootloader/include'), '-I' + str(lc / 'update_lib/include'),
                        *[str(ROOT / 'Core/Src' / n) for n in names], str(ROOT / 'Bootloader/storage_dispatch.c'),
                        str(lc / 'bootloader/src/persist.c'), str(lc / 'bootloader/src/crc32.c'), str(p / 'steps.c'),
                        '-o', str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_case(self, name):
        out = subprocess.run([str(self.exe), name], capture_output=True, text=True, timeout=60)
        self.assertEqual(out.returncode, 0, out.stdout + out.stderr)
        self.assertIn('PASS', out.stdout)

    def test_backward_4_h_step(self):
        self.run_case('step_back')

    def test_forward_3_8_h_step(self):
        self.run_case('step_forward')

    def test_newer_epoch_request_while_reanchoring(self):
        self.run_case('reanchor')

    def test_no_network_time_refuses_station_requests(self):
        self.run_case('no_time')


if __name__ == '__main__':
    unittest.main()
