"""Actual managed-state admission; floor/SDK boundaries modeled explicitly.

Verifies inactive FSTATE source authority while preserving other controller
message policies. This is not network delivery or physical owner evidence.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
def function(t,name):
 i=t.index(name+'(');start=t.rfind('\n',0,i)+1;j=t.index('{',i);depth=1;k=j+1
 while depth:depth+=(t[k]=='{')-(t[k]=='}');k+=1
 return t[start:k]
class ManagedAuthorityTests(unittest.TestCase):
 def test_actual_fstate_only_authority_and_unchanged_controller_policy(self):
  text=(ROOT/'Core/Src/board_recovery_transport.c').read_text()
  body=function(text,'board_recovery_managed_update_allowed')
  code='''#include <stdbool.h>
  #include <stdint.h>
  #include <stddef.h>
  #include <string.h>
  #include <assert.h>
  #include <stdio.h>
  #include <stdlib.h>
  typedef int SedsRouter;typedef int SedsResult;
  #define SEDS_OK 0
  static int router;static SedsRouter*recovery_router=&router;
  #define SEDS_DT_FLIGHT_STATE 11U
  #define SEDS_DT_FLIGHT_COMMAND 12U
  #define SEDS_DT_ASCENT_BIASES 13U
  typedef struct{uint32_t ty;const char*sender;size_t sender_len;uint64_t timestamp;}SedsPacketView;
  static bool ready=true,healthy=true,active=false,time_ok=true;static uint64_t now=20000;
  bool persistent_store_controls_ready(void){return ready;}
  uint8_t board_recovery_floor_is_healthy(void){return healthy;}
  uint8_t board_recovery_floor_active(void){return active;}
  uint8_t board_recovery_command_time_allowed(uint64_t source){return time_ok&&source>100;}
  SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){assert(r);*out=now;return 0;}
  bool seds_router_sender_matches_hostname(SedsRouter*r,const char*s,size_t len,const char*name,size_t n){assert(r);return s&&len==n&&!memcmp(s,name,n);}
  '''+function(text,'authorized_primary')+'\n'+function(text,'board_recovery_command_is_stale')+'\n'+body+'''
  int main(int argc,char**argv){assert(argc==2);SedsPacketView p={.ty=SEDS_DT_FLIGHT_STATE,.sender="VB",.sender_len=2,.timestamp=9000};
   if(!strcmp(argv[1],"inactive_nonGS")){assert(!board_recovery_managed_update_allowed(&p));return 0;}
   const char*senders[]={"GS","RF","VB","FC","RRLS"};unsigned cases=0;
   for(unsigned type=11;type<=13;type++)for(unsigned ac=0;ac<2;ac++)for(unsigned h=0;h<2;h++)for(unsigned rr=0;rr<2;rr++)for(unsigned src=0;src<5;src++)for(unsigned stale=0;stale<2;stale++){
   p.ty=type;active=ac;healthy=h;ready=rr;p.sender=senders[src];p.sender_len=strlen(p.sender);p.timestamp=stale?4999:5000;
   bool allowed=board_recovery_managed_update_allowed(&p);bool expected=healthy&&READY_GUARD&&(!active?(type!=SEDS_DT_FLIGHT_STATE||src==0):(src==0&&!stale));assert(allowed==expected);cases++;
   }
   healthy=ready=true;active=false;assert(board_recovery_managed_update_allowed(NULL));recovery_router=NULL;p.ty=SEDS_DT_FLIGHT_STATE;p.sender="GS";p.sender_len=2;assert(!board_recovery_managed_update_allowed(&p));p.ty=SEDS_DT_FLIGHT_COMMAND;assert(board_recovery_managed_update_allowed(&p));printf("actual authority matrix %u PASS\\n",cases);return 0;}
  '''.replace('READY_GUARD','true' if ROOT.name in ['ActuatorBoard26','ValveBoard26'] else 'ready')
  with tempfile.TemporaryDirectory() as tmp:
   d=Path(tmp);p=d/'probe.c';p.write_text(code);exe=d/'probe'
   subprocess.run(['cc','-std=c17','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-g',str(p),'-o',str(exe)],check=True,capture_output=True)
   for case in ['inactive_nonGS','matrix']:
    subprocess.run([str(exe),case],check=True,capture_output=True)
