"""Router construction must have a live clock and retain its chosen origin."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class RouterClockTests(unittest.TestCase):
    def test_clock_advances_before_router_pointer_is_published(self):
        source = (ROOT / 'Core/Src/telemetry.c').read_text()
        start = source.index('node_now_since_ms(void *user)')
        start = source.rfind('static ', 0, start)
        end = source.index('\n}', start) + 2
        code = r'''#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#define UNUSED_FUNCTION
 typedef struct {void *r; unsigned created;uint64_t start_time;} RouterState;
 static RouterState g_router={NULL,0,100};
 static uint64_t tick=100;
 static uint64_t tx_raw_now_ms_locked(void){return tick;}
'''+source[start:end]+r'''
int main(void){
 assert(node_now_since_ms(NULL)==0);
 tick=128;assert(node_now_since_ms(NULL)==28);
 g_router.r=(void*)1;g_router.created=1;
 assert(node_now_since_ms(NULL)==28);
 tick=129;assert(node_now_since_ms(NULL)==29);
 g_router.r=NULL;assert(node_now_since_ms(NULL)==29);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            exe=Path(tmp)/'clock'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-x','c','-','-o',str(exe)],input=code,text=True,check=True)
            subprocess.run([str(exe)],check=True)

    def test_origin_is_set_before_construction_and_never_reset_after(self):
        source = (ROOT / 'Core/Src/telemetry.c').read_text()
        constructor = source.index('r = seds_router_new')
        epochs = list(re.finditer(r'g_router\.start_time\s*=', source))
        self.assertEqual(len(epochs),1)
        self.assertLess(epochs[0].start(),constructor)

if __name__=='__main__':
    unittest.main()
