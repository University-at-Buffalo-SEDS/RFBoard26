"""Station FLIGHT_STATE writes apply in (epoch, stamp) order across clock steps.

Bench (2026-11-16): after the GS clock stepped back, the last applied station
FLIGHT_STATE stamp refused every later write (Launch, Reset-to-Idle) until the
FC rebooted. The actual flight_state_cache_order() runs on views carrying the
SEDSnet 6 time-base epoch: a write made after a step in either direction is
newer than every earlier one, a late copy stamped before the step never
applies after it, and Aborted (15) always applies.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest
from router_lifecycle_fixture import function
from network_stamp_model import header

ROOT = Path(__file__).resolve().parents[1]

CODE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
typedef struct {const char*sender;size_t sender_len;uint64_t timestamp;const uint8_t*payload;size_t payload_len;uint16_t time_epoch;} SedsPacketView;
'''
MAIN = r'''
#define T0 UINT64_C(1794183600000)
#define H4 UINT64_C(14400000)
static uint8_t state;
/* The handler's sequence: refused when older, else applied and remembered. */
static bool apply(const char*sender,uint8_t value,uint16_t epoch,uint64_t stamp){
 uint8_t v=value;SedsPacketView p={sender,2,stamp,&v,1,epoch};
 if(flight_state_cache_order(&p,false))return false;
 state=value;(void)flight_state_cache_order(&p,true);return true;
}
int main(void){
 assert(apply("GS",2,0,T0));
 /* GS clock steps back 4 h: epoch 1. Launch and Idle on the new clock apply. */
 assert(apply("GS",1,1,T0-H4+1000)&&state==1);
 assert(apply("GS",7,1,T0-H4+2000)&&state==7);
 /* Late copies stamped before the step never step it back. */
 assert(!apply("GS",2,0,T0)&&state==7);
 assert(!apply("GS",1,0,T0+500)&&state==7);
 /* An exact repeat is not applied again; an older write in the epoch is refused. */
 assert(!apply("GS",7,1,T0-H4+2000));assert(!apply("GS",1,1,T0-H4+1500)&&state==7);
 /* Aborted always applies, even stamped before the step. */
 assert(apply("GS",15,0,T0-1)&&state==15);
 /* A forward 3.8 h step (epoch 2): newer writes apply, epoch-1 copies do not. */
 assert(apply("GS",2,2,T0+13680000)&&state==2);
 assert(!apply("GS",1,1,T0-H4+3000)&&state==2);
 /* The RRLS is ordered separately. */
 assert(apply("RR",1,0,T0-5)&&state==1);
 puts("flight state order across steps PASS");return 0;
}
'''


class FlightStateOrderEpochs(unittest.TestCase):
    def test_writes_ordered_by_epoch_across_steps(self):
        cache = (ROOT / 'Core/Src/flight_state_cache.c').read_text()
        code = CODE + header() + 'volatile uint32_t g_flight_state_writes_out_of_order;\n' + \
            function(cache, 'flight_state_cache_order') + MAIN
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / 'order'
            build = subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                                    '-fsanitize=address,undefined', '-x', 'c', '-', '-o', str(exe)],
                                   input=code, text=True, capture_output=True)
            self.assertEqual(build.returncode, 0, build.stderr)
            run = subprocess.run([str(exe)], text=True, capture_output=True)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            self.assertIn('PASS', run.stdout)


if __name__ == '__main__':
    unittest.main()
