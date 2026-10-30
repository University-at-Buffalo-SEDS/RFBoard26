"""Execute actual temperature state code with ADC calls stubbed, not its logic."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
def function(source,signature):
 start=source.index(signature);brace=source.index('{',start);depth=1;end=brace+1
 while depth:
  depth+=(source[end]=='{')-(source[end]=='}');end+=1
 return source[start:end]+'\n'
def run_lifecycle(source,include,start=0,init_fails=False):
 prefix=source[source.index('#define ADC_COMMON '):source.index('static bool due(')]
 if 'static uint8_t init_state;' in source:prefix+='static uint8_t init_state;\n'
 functions=''.join(function(source,s) for s in ['static bool due(','static void publish(','uint32_t board_temperature_health_flags(','void board_temperature_poll('])
 harness=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "board_temperature.h"
#include "board_temperature_math.h"
#define LL_ADC_CHANNEL_TEMPSENSOR_ADC1 1U
#define LL_ADC_CHANNEL_VREFINT 2U
static const uint16_t cal_ref=1500,cal_low=1200,cal_high=1600;
#define VREFINT_CAL_ADDR (&cal_ref)
#define TEMPSENSOR_CAL1_ADDR (&cal_low)
#define TEMPSENSOR_CAL2_ADDR (&cal_high)
#define TEMPSENSOR_CAL1_TEMP 30
#define TEMPSENSOR_CAL2_TEMP 130
static uint32_t tick,init_calls,reads,observed;
static unsigned mask;
static bool fail_init,fail_read,reenter;
uint32_t HAL_GetTick(void){return tick;}
unsigned __get_PRIMASK(void){return mask;}
void __disable_irq(void){mask=1;}
void __set_PRIMASK(unsigned value){mask=value;}
static bool initialize(void){assert(!mask);++init_calls;return !fail_init;}
static bool read_channel(uint32_t channel,uint32_t *out){
 assert(!mask);++reads;observed=board_temperature_health_flags();
 if(reenter){reenter=false;board_temperature_poll();}
 if(fail_read)return false;
 *out=channel==LL_ADC_CHANNEL_VREFINT?1500:1200;return true;
}
'''+prefix+functions+r'''
int main(void){
 const uint32_t start=START;
 tick=start;fail_init=INIT_FAIL;board_temperature_poll();assert(!mask&&init_calls==1);
 if(fail_init){
  assert(!reads&&!board_temperature_health_flags());
  for(unsigned i=1;i<=20000;++i){tick=start+i;board_temperature_poll();}
  assert(init_calls==1&&!reads&&!board_temperature_health_flags());
  printf("init failure cached; periodic retries bounded; mask=%u\n",mask);return 0;
 }
 assert(reads==2);assert(board_temperature_health_flags()==board_temperature_pack(3000,0));
 for(unsigned i=1;i<5000;++i){tick=start+i;board_temperature_poll();}
 assert(reads==2&&init_calls==1);
 mask=1;assert(board_temperature_health_flags()==board_temperature_pack(3000,4999)&&mask==1);mask=0;
 tick=start+5000;fail_read=true;board_temperature_poll();
 assert(reads==3&&init_calls==1&&!board_temperature_health_flags());
 assert(observed==board_temperature_pack(3000,5000)); /* No false-new timestamp in flight. */
 tick=start+9999;board_temperature_poll();assert(reads==3);
 tick=start+10000;fail_read=false;reenter=true;board_temperature_poll();
 assert(reads==5&&init_calls==1);assert(board_temperature_health_flags()==board_temperature_pack(3000,0));
 tick=start+16001;assert(!board_temperature_health_flags());assert(!mask);
 printf("first0/wrap/failure/recovery/reentry/staleness; reads=%u init=%u mask=%u\n",reads,init_calls,mask);
 return 0;
}
'''
 harness=harness.replace('START',str(start)+'U').replace('INIT_FAIL','true' if init_fails else 'false')
 with tempfile.TemporaryDirectory()as tmp:
  exe=Path(tmp)/'state'
  subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(include),'-x','c','-','-o',str(exe)],input=harness,text=True,check=True)
  return subprocess.check_output([str(exe)],text=True)
class TemperatureStateTests(unittest.TestCase):
 def test_first_tick_zero_failed_read_staleness_and_reentrant_retry(self):
  run_lifecycle((ROOT/'Core/Src/board_temperature.c').read_text(),ROOT/'Core/Inc')
 def test_wraparound_preserves_five_second_cadence_and_age(self):
  run_lifecycle((ROOT/'Core/Src/board_temperature.c').read_text(),ROOT/'Core/Inc',0xfffffc17)
 def test_failed_initialization_does_not_reinitialize_or_report_valid(self):
  run_lifecycle((ROOT/'Core/Src/board_temperature.c').read_text(),ROOT/'Core/Inc',0,True)
