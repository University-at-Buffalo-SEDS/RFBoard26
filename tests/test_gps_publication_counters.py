"""GPS publication probes count the real SDK result, including refusals."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]
class GpsPublicationCounterTests(unittest.TestCase):
    def test_attempt_success_and_invalid_input_are_distinct(self):
        source=(ROOT/'Core/Src/telemetry.c').read_text()
        start=source.index('SedsResult log_telemetry_asynchronous(')
        end=source.index('SedsResult log_telemetry_string_asynchronous(',start)
        body=source[start:end]
        scaffold=r'''
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#define TELEMETRY_ENABLED 1
#define SEDS_DT_GPS_DATA 0x1234U
typedef uint32_t SedsDataType;
typedef int SedsResult;
enum { SEDS_OK=0,SEDS_ERR=-1,SEDS_BAD_ARG=-2,SEDS_IO=-3 };
static struct { void *r; } g_router;
static uint32_t g_sim_gps_publish_attempts,g_sim_gps_publish_ok,calls,init_calls;
static int sdk_result,init_result;
static SedsResult init_telemetry_router(void) {
 ++init_calls; if(init_result==0)g_router.r=(void*)1; return init_result;
}
static int guess_kind_from_elem_size(size_t size) {assert(size==4);return 2;}
static SedsResult seds_router_log_typed(void *router,SedsDataType type,
 const void *data,size_t count,size_t size,int kind) {
 assert(router && data && count==3 && size==4 && kind==2);
 assert(type==SEDS_DT_GPS_DATA || type==99);++calls;return sdk_result;
}
'''
        main=r'''
int main(void) {
 float gps[3]={43,-78.78f,300};g_router.r=(void*)1;
 assert(log_telemetry_asynchronous(SEDS_DT_GPS_DATA,gps,3,4)==SEDS_OK);
 assert(g_sim_gps_publish_attempts==1 && g_sim_gps_publish_ok==1 && calls==1);
 sdk_result=SEDS_IO;
 assert(log_telemetry_asynchronous(SEDS_DT_GPS_DATA,gps,3,4)==SEDS_IO);
 assert(g_sim_gps_publish_attempts==2 && g_sim_gps_publish_ok==1 && calls==2);
 assert(log_telemetry_asynchronous(99,gps,3,4)==SEDS_IO);
 assert(g_sim_gps_publish_attempts==2 && g_sim_gps_publish_ok==1 && calls==3);
 assert(log_telemetry_asynchronous(SEDS_DT_GPS_DATA,0,3,4)==SEDS_BAD_ARG);
 assert(g_sim_gps_publish_attempts==2 && calls==3);
 g_router.r=0;init_result=SEDS_ERR;
 assert(log_telemetry_asynchronous(SEDS_DT_GPS_DATA,gps,3,4)==SEDS_ERR);
 assert(init_calls==1 && g_sim_gps_publish_attempts==2 && calls==3);
 init_result=0;sdk_result=SEDS_OK;
 assert(log_telemetry_asynchronous(SEDS_DT_GPS_DATA,gps,3,4)==SEDS_OK);
 assert(init_calls==2 && g_sim_gps_publish_attempts==3 && g_sim_gps_publish_ok==2 && calls==4);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            file=Path(tmp)/'gps.c';file.write_text(scaffold+body+main);exe=Path(tmp)/'gps'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(file),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
