"""Exercise actual cosmetic-setting callbacks against transient flash failures."""
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
int main(void){
 (void)irq_mask;
 SedsRouter router=0;assert(av_bay_underglow_init(&router)==SEDS_OK);assert(callback);
 assert(update(1)==SEDS_OK);assert(writes==1&&stored==1&&pin==1);
 assert(update(1)==SEDS_OK);assert(writes==1); /* avoid successful-value flash wear */
 fail_write=1;
 assert(update(0)==SEDS_HANDLER_ERROR);assert(writes==2&&stored==1&&pin==0);
 assert(update(0)==SEDS_HANDLER_ERROR);assert(writes==3&&stored==1); /* no false receipt */
 fail_write=0;
 assert(update(0)==SEDS_OK);assert(writes==4&&stored==0); /* identical retry repairs flash */
 assert(update(0)==SEDS_OK);assert(writes==4);
 assert(g_av_bay_underglow_persist_errors==2&&g_av_bay_underglow_persist_writes==2);
}
'''

class UnderglowPersistenceTests(unittest.TestCase):
    def test_failed_changed_write_retries_until_durable(self):
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
