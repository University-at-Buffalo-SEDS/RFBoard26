from pathlib import Path
import subprocess, tempfile, unittest
ROOT=Path(__file__).resolve().parents[1]
class TemperatureTests(unittest.TestCase):
 def test_factory_ratio_and_wire_backward_compatibility(self):
  c=r"""
#include "board_temperature_math.h"
#include <assert.h>
#include <stddef.h>
int main(void) {
 int16_t c=0;
 /* 12-bit factory codes; supply and measured resolution cancel in ratio. */
 assert(board_temperature_convert(1200,1500,1500,1200,1600,30,130,&c)&&c==3000);
 assert(board_temperature_convert(2800,3000,1500,1200,1600,30,130,&c)&&c==8000);
 assert(board_temperature_convert(1100,1500,1500,1200,1600,30,130,&c)&&c==500);
 /* U585 factory uses 14-bit calibration, same ratio at lower ADC scale. */
 assert(board_temperature_convert(2600,3000,6000,4800,6400,30,130,&c)&&c==5500);
 assert(!board_temperature_convert(1200,0,1500,1200,1600,30,130,&c));
 assert(!board_temperature_convert(1200,1500,1500,1600,1200,30,130,&c));
 assert(!board_temperature_convert(1200,1500,65535,1200,1600,30,130,&c));
 assert(!board_temperature_convert(4000,1500,1500,1200,1600,30,130,&c));
 assert(!board_temperature_convert(1200,1500,1500,1200,1600,30,130,NULL));
 uint32_t legacy=0xff, flags=legacy|board_temperature_pack(-1234,101);
 assert((flags&0xff)==legacy); /* every previous counter's validity unchanged */
 assert(flags&0x100);assert((int16_t)(flags>>16)==-1234);
 assert(((flags>>9)&127)==2); /* conservative age rounds upward */
 assert(board_temperature_pack(15000,6000));
 assert(!board_temperature_pack(15001,0));assert(!board_temperature_pack(2500,6001));
}
"""
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);(p/'test.c').write_text(c)
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=undefined','-I'+str(ROOT/'Core/Inc'),str(p/'test.c'),'-o',str(p/'test')],check=True)
   subprocess.run([str(p/'test')],check=True)
