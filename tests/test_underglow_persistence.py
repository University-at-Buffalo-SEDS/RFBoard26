"""Underglow is cosmetic: kept in RAM and written to flash only after it settles.

Bench, 2026-11-10: one flash record per toggle filled the FC store with about
210 underglow records. A toggle must drive the LEDs and be acknowledged at once,
and reach flash at most once a minute, only after the value has settled.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

STUB = r'''
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>
typedef int SedsResult;
typedef int SedsRouter;
typedef int launchcore_persist_status_t;
typedef struct { uint32_t ty; const uint8_t *payload; size_t payload_len; } SedsPacketView;
#define SEDS_DT_AV_BAY_UNDERGLOW 7
#define SEDS_OK 0
#define SEDS_HANDLER_ERROR 1
#define SEDS_BAD_ARG 2
#define LAUNCHCORE_PERSIST_OK 0
#define LAUNCHCORE_PERSIST_NOT_FOUND 1
#define LAUNCHCORE_PERSIST_STORAGE_ERROR 2
#define LED2_PORT 0
#define LED2_PIN 0
#define BLUE_LED_GPIO_Port 0
#define BLUE_LED_Pin 0
#define BLUE_LEDS_GPIO_Port 0
#define BLUE_LEDS_Pin 0
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
volatile uint32_t g_telemetry_discovery_seen;
static uint32_t tick, irq_mask;
#define __get_PRIMASK() irq_mask
#define __disable_irq() ((void)(irq_mask=1))
#define __set_PRIMASK(m) ((void)(irq_mask=(m)))
static uint32_t HAL_GetTick(void) { return tick; }
static int pin, fail_write;
static unsigned writes;
static uint8_t stored;
static void HAL_GPIO_WritePin(int port,int number,int value) { (void)port;(void)number;pin=value; }
static int persistent_store_init(void) { return 0; }
static int persistent_store_get(uint32_t k,void *v,size_t *n) { (void)k;(void)v;(void)n;return 1; }
static int persistent_store_set(uint32_t k,const void *v,size_t n) {
 (void)k;assert(n==1);++writes;
 if(fail_write)return LAUNCHCORE_PERSIST_STORAGE_ERROR;
 stored=*(const uint8_t*)v;return 0;
}
static SedsResult (*callback)(const SedsPacketView*,void*);
static int seds_router_enable_network_variable(SedsRouter *r,int ty,bool a,bool b) { (void)r;(void)ty;(void)a;(void)b;return 0; }
static int seds_router_on_network_variable_update(SedsRouter *r,int ty,SedsResult (*fn)(const SedsPacketView*,void*),void *u) { (void)r;(void)ty;(void)u;callback=fn;return 0; }
static int seds_router_request_managed_variable(SedsRouter *r,int ty) { (void)r;(void)ty;return 0; }
'''
TEST = r'''
static SedsResult update(uint8_t value){SedsPacketView p={SEDS_DT_AV_BAY_UNDERGLOW,&value,1};return callback(&p,NULL);}
static void at(uint32_t ms){SedsRouter r=0;tick=ms;assert(av_bay_underglow_poll(&r)==SEDS_OK);}
int main(void){
 (void)irq_mask;
 SedsRouter router=0;assert(av_bay_underglow_init(&router)==SEDS_OK);assert(callback);
 /* A toggle drives the LEDs and is acknowledged at once; flash waits for it to settle. */
 tick=1000;assert(update(1)==SEDS_OK&&pin==1&&writes==0);
 at(1000+UNDERGLOW_PERSIST_SETTLE_MS-1U);assert(writes==0);
 at(1000+UNDERGLOW_PERSIST_SETTLE_MS);assert(writes==1&&stored==1);
 assert(update(1)==SEDS_OK);at(tick+UNDERGLOW_PERSIST_SETTLE_MS);assert(writes==1); /* unchanged: no wear */
 /* Five thousand bench toggles, 10 ms apart: no flash write while they run. */
 uint32_t t0=tick;
 for(unsigned i=1;i<=5000;i++){tick=t0+i*10U;assert(update((uint8_t)(i&1U))==SEDS_OK&&pin==(int)(i&1U));at(tick);}
 assert(writes==1&&stored==1&&pin==0);
 /* The final value is written once it settles, at most once a minute. */
 assert(update(0)==SEDS_OK&&pin==0);uint32_t last=tick;
 at(last+UNDERGLOW_PERSIST_SETTLE_MS);
 const uint32_t first_write=1000+UNDERGLOW_PERSIST_SETTLE_MS;
 if(last+UNDERGLOW_PERSIST_SETTLE_MS<first_write+UNDERGLOW_PERSIST_MIN_INTERVAL_MS){
  assert(writes==1);at(first_write+UNDERGLOW_PERSIST_MIN_INTERVAL_MS-1U);assert(writes==1);
  at(first_write+UNDERGLOW_PERSIST_MIN_INTERVAL_MS);
 }
 assert(writes==2&&stored==0);
 /* An hour of toggles every 3 s: bounded by the minute rate, not one per toggle. */
 t0=tick;const unsigned before=writes;
 for(unsigned i=1;i<=1200;i++){tick=t0+i*3000U;assert(update((uint8_t)(i&1U))==SEDS_OK);at(tick);at(tick+UNDERGLOW_PERSIST_SETTLE_MS);}
 assert(writes-before<=61U&&writes-before>=1U);
 /* A failed write is acknowledged and retried later, never from the callback. */
 at(tick+UNDERGLOW_PERSIST_MIN_INTERVAL_MS);const unsigned settled=writes;const uint8_t target=stored==1?0:1;
 fail_write=1;tick+=UNDERGLOW_PERSIST_MIN_INTERVAL_MS;assert(update(target)==SEDS_OK&&writes==settled);
 at(tick+UNDERGLOW_PERSIST_SETTLE_MS);assert(writes==settled+1U&&stored!=target&&g_av_bay_underglow_persist_errors==1);
 fail_write=0;at(tick+UNDERGLOW_PERSIST_MIN_INTERVAL_MS-1U);assert(writes==settled+1U);
 at(tick+UNDERGLOW_PERSIST_MIN_INTERVAL_MS);assert(writes==settled+2U&&stored==target);
 assert(g_av_bay_underglow_persist_writes==writes-1U);
}
'''

class UnderglowPersistenceTests(unittest.TestCase):
    def test_toggles_reach_flash_only_after_settling_and_rate_limited(self):
        source = (ROOT / 'Core/Src/av_bay_underglow.c').read_text()
        source = '\n'.join(line for line in source.splitlines()
                           if not line.startswith('#include "'))
        with tempfile.TemporaryDirectory(prefix='underglow-persistence-') as temp:
            executable = str(Path(temp) / 'test')
            subprocess.run(['cc', '-DTELEMETRY_ENABLED=1', '-std=c11', '-Wall',
                            '-Wextra', '-Werror', '-x', 'c', '-', '-o', executable],
                           input=STUB + source + TEST, text=True, check=True)
            subprocess.run([executable], check=True)

if __name__ == '__main__':
    unittest.main()
