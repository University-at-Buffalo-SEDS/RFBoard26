"""Station commands are admitted again within seconds of a GS clock step.

Bench (2026-11-16): the GS network clock stepped about 3 h 49 min back.
The station clock the AB judges station stamps by only moved forward, and
the persisted recovery floor (the last Reset's station stamp) was then
hours ahead of every new request, so valve, QD, FLIGHT_STATE and Launch
requests were refused silently while Abort still went through.

The actual station clock, freshness, floor and RAM cutoff code runs against
a GS that sends a heartbeat every second and steps its clock by 4 h in
either direction, while the board's own network clock keeps its old value
(as a slewing follower would). Commands stamped on the new GS clock must be
admitted within a few seconds; a request stamped before the Reset must
never be, and a single late heartbeat must not move the station clock back.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from router_lifecycle_fixture import function

ROOT = Path(__file__).resolve().parents[1]


def block(source, first, last):
    return re.search(re.escape(first) + r".*?" + re.escape(last), source, re.S).group()


def harness():
    transport = (ROOT / "Core/Src/board_recovery_transport.c").read_text()
    scaffold = r"""
#include <assert.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define TX_SUCCESS 0U
#define BOARD_RECOVERY_STATION_STEP_BACK_MS UINT64_C(60000)
typedef unsigned long ULONG;typedef unsigned UINT;
typedef int SedsResult;typedef int SedsRouter;
enum {SEDS_OK=0,SEDS_IO=-14,SEDS_DT_HEARTBEAT=106,SEDS_DT_VALVE=5};
typedef struct {int ty;const char*sender;size_t sender_len;uint64_t timestamp;} SedsPacketView;
#define T0 UINT64_C(1794183600000)
static SedsRouter cookie;static SedsRouter*recovery_router=&cookie;
static ULONG ticks;static ULONG tx_time_get(void){return ticks;}
static uint64_t net_fixed=T0;
static SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){(void)r;*out=net_fixed+ticks;return SEDS_OK;}
static uint8_t authorized_primary(SedsRouter*r,const SedsPacketView*p){(void)r;return p&&p->sender_len==2&&!memcmp(p->sender,"GS",2);}
/* Persisted floor: the last Reset, stamped at T0 on the old GS clock. */
static uint64_t floor_ts=T0;
static uint8_t board_recovery_command_time_allowed(uint64_t s){return s>floor_ts;}
static uint8_t board_recovery_floor_active(void){return 1U;}
static uint8_t board_recovery_floor_read_cutoff(uint64_t*c){*c=floor_ts;return 1U;}
static uint8_t board_recovery_floor_cached_cutoff(uint64_t*c){*c=floor_ts;return 1U;}
static uint64_t cutoff;
enum {SEDS_DT_FLIGHT_STATE=11};static unsigned invalidated;
static SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*r,int ty,bool*out){(void)out;assert(r==&cookie&&ty==SEDS_DT_FLIGHT_STATE);invalidated++;return SEDS_OK;}
uint64_t board_recovery_station_now(SedsRouter*router);
uint8_t board_recovery_station_behind(uint64_t mark,uint64_t station);
"""
    pieces = [
        scaffold,
        block(transport, "static uint64_t station_utc;", "static uint8_t station_stepped_back;"),
        *(function(transport, n) for n in (
            "station_estimate", "station_store",
            "board_recovery_station_behind", "board_recovery_observe_station_time",
            "board_recovery_station_now", "board_recovery_command_is_stale")),
    ]
    return "\n".join(pieces) + MAIN


MAIN = r"""
static int64_t gs_offset;
static uint64_t gs_now(void){return (uint64_t)((int64_t)(T0+ticks)+gs_offset);}
static void heartbeat(void){SedsPacketView hb={SEDS_DT_HEARTBEAT,"GS",2,gs_now()};board_recovery_observe_station_time(&cookie,&hb);}
/* A station FLIGHT_STATE/command as the board admits it: the stale check. */
static bool admitted(uint64_t stamp){
 SedsPacketView p={SEDS_DT_VALVE,"GS",2,stamp};
 return !board_recovery_command_is_stale(&p);
}
static void run(int64_t step){
 ticks=1;gs_offset=0;cutoff=0;invalidated=0;
 /* Reset at T0 + 1 s on the old clock; GS clock then running. */
 floor_ts=T0+1000;
 for(;ticks<10000;ticks++)if(ticks%1000==500)heartbeat();
 assert(admitted(gs_now()));
 const uint64_t before_reset=floor_ts-500;
 assert(!admitted(before_reset));
 gs_offset+=step;
 ULONG accepted_at=0;
 for(ULONG end=ticks+20000;ticks<end;ticks++){
  if(ticks%1000==500)heartbeat();
  if(ticks%100==0&&!accepted_at&&admitted(gs_now()))accepted_at=ticks;
  /* A copy stamped before the Reset is never admitted. */
  assert(!admitted(before_reset));
 }
 assert(accepted_at);
 /* A step back also drops the SDK's FLIGHT_STATE stamped on the old clock. */
 assert(step>0||invalidated>0);
 const ULONG took=accepted_at-(ULONG)10000;
 printf("step %+lld ms: accepted after %lu ms\n",(long long)step,took);
 assert(took<=3000U);
 assert(admitted(gs_now()));
 /* Stamps on the old clock, far from the new one, stay refused. */
 assert(!admitted(gs_now()-(uint64_t)(step>0?step:0)+(step<0?(uint64_t)(-step):0)));
}
static void single_late_heartbeat(void){
 ticks=1;gs_offset=0;cutoff=0;floor_ts=T0+1000;invalidated=0;
 station_store(0U);back_held=0U;
 for(;ticks<5000;ticks++)if(ticks%1000==500)heartbeat();
 const uint64_t before=board_recovery_station_now(&cookie);
 SedsPacketView late={SEDS_DT_HEARTBEAT,"GS",2,before-600000};board_recovery_observe_station_time(&cookie,&late);
 ticks++;heartbeat();
 assert(board_recovery_station_now(&cookie)>=before);
#ifndef ANY_PACKET
 /* Two late non-heartbeat packets never re-anchor either. */
 SedsPacketView cmd={SEDS_DT_VALVE,"GS",2,before-600000};board_recovery_observe_station_time(&cookie,&cmd);
 ticks+=1000;cmd.timestamp+=1000;board_recovery_observe_station_time(&cookie,&cmd);
 assert(board_recovery_station_now(&cookie)>=before);
#endif
 /* An unauthorized sender never moves it. */
 SedsPacketView other={SEDS_DT_HEARTBEAT,"VB",2,before-600000};board_recovery_observe_station_time(&cookie,&other);
 ticks+=1000;other.timestamp+=1000;board_recovery_observe_station_time(&cookie,&other);
 assert(board_recovery_station_now(&cookie)>=before);
 assert(!invalidated);
}
int main(void){
 run(-INT64_C(14400000));
 run(INT64_C(14400000));
 single_late_heartbeat();
 return 0;
}
"""


class StationClockStep(unittest.TestCase):
    def test_commands_admitted_within_seconds_of_a_4_h_step(self):
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / "step"
            transport = (ROOT / "Core/Src/board_recovery_transport.c").read_text()
            # Boards without a heartbeat endpoint re-anchor on any GS packet.
            extra = [] if "packet->ty!=SEDS_DT_HEARTBEAT" in transport else ["-DANY_PACKET"]
            build = subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror"] + extra + [
                                    "-Wno-unused-function", "-fsanitize=address,undefined",
                                    "-x", "c", "-", "-o", str(exe)],
                                   input=harness(), text=True, capture_output=True)
            self.assertEqual(build.returncode, 0, build.stderr)
            run = subprocess.run([str(exe)], text=True, capture_output=True)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)


if __name__ == "__main__":
    unittest.main()
