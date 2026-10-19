from pathlib import Path
import re
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]

def function(source,name):
    match=re.search(r'static\s+(?:uint32_t|SedsResult)\s+'+name+r'\([^;]*?\)\s*\{',source)
    assert match,name
    end=match.end();depth=1
    while depth:depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[match.start():end]

class CanOwnershipTests(unittest.TestCase):
    def test_priority_bands_and_complete_frame_ownership_result(self):
        source=(ROOT/'Core/Src/telemetry.c').read_text()
        code=(ROOT/'Core/Inc/board_can_priority.h').read_text()+r'''
#include <assert.h>
typedef int SedsResult;
enum{SEDS_OK=0,SEDS_IO=8,HAL_OK=0,HAL_BUSY=2};
#define SEDS_DT_HEARTBEAT 120
#define TELEMETRY_FLIGHT_CAN_ID 0x101U
#define TELEMETRY_FLIGHT_HEARTBEAT_CAN_ID 0x001U
static uint32_t dtype,observed;static unsigned calls;static int result;
static uint32_t sim_probe_packed_data_type(const uint8_t *p,size_t n){(void)p;(void)n;return dtype;}
static int can_bus_send_large(const uint8_t *p,size_t n,uint32_t id){assert(p&&n);calls++;observed=id;return result;}
'''+function(source,'telemetry_flight_can_id')+function(source,'telemetry_send_can_packet')+r'''
int main(void){
 uint8_t opaque[3][8]={{'S','D','T',1},{'S','D','T',2},{'S','D',7,1}};
 for(unsigned i=0;i<3;i++)for(unsigned p=0;p<256;p++)
  assert(telemetry_flight_can_id(opaque[i],8,p)==((i<2&&p>=200)?0x81:(p>=200?1:0x101)));
 dtype=120;assert(telemetry_flight_can_id(opaque[2],8,0)==1);
 dtype=10;assert(telemetry_flight_can_id(opaque[2],8,0)==0x81);
 dtype=0;result=HAL_BUSY;
 assert(telemetry_send_can_packet(opaque[2],8,255)==SEDS_IO&&calls==1&&observed==1);
 result=HAL_OK;assert(telemetry_send_can_packet(opaque[2],8,0)==SEDS_OK&&calls==2&&observed==0x101);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            exe=Path(tmp)/'owner'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=code,text=True,check=True)
            subprocess.run([str(exe)],check=True)
