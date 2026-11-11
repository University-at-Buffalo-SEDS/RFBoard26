"""Thousands of setting changes leave this board's persistent store healthy.

Bench, 2026-11-10: the FC store filled with about 210 underglow records written
on every toggle, plus buzzer records. This drives the actual LaunchCore journal
(persist.c) on this board's own store geometry with the board's own records:
thousands of flight-state changes, and on boards with underglow, ten
thousand toggles through the actual underglow module. The journal must stay
HEALTHY, every other record must survive, and cosmetic toggles must reach flash
only at the module's settled, rate-limited pace.
"""
import os
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
UNDERGLOW = ROOT / "Core/Src/av_bay_underglow.c"


def board_value(suffix):
    text = (ROOT / "Bootloader/board_config.h").read_text()
    match = re.search(r"(?m)^#define\s+\w*" + suffix + r"\s+(0x[0-9A-Fa-f]+|\d+)u?\s*$", text)
    return int(match.group(1), 0)


# The strict journal rescans a bank on every write, so a larger bank costs more
# per change: 10,000 changes on a 2 KiB bank, at least 3,000 (still several
# compactions of each bank) on larger ones.
FSTATE_CHANGES = int(os.environ.get("PERSIST_ENDURANCE_CHANGES", "0")) or max(
    3000, 10000 * 2048 // max(2048, board_value("PERSIST_SIZE") // 2))


def write_size():
    text = (ROOT / "Bootloader/storage_internal_flash.c").read_text()
    return int(re.search(r"\.persistent_data_write_size\s*=\s*(\d+)u?", text).group(1))


def launchcore_source():
    from native_sdk import configured_dependency_source
    return configured_dependency_source(ROOT, "sedslaunchcore", "SDK_FREE_LAUNCHCORE_SOURCE")


# Records every board keeps (identity, recovery floor, commissioning/bootstrap
# marker), then the board-specific ones, each at its stored size.
RECORDS = [(0x42524745, 8), (0x4252464C, 16), (0x42525043, 40)]
# Keyed by the module that owns them, never by the checkout's directory name.
EXTRA = {"fill_settings.c": [(0x46524456, 32)], "daq_calibration.c": [(0x44414341, 64), (0x4441494E, 4)],
         "rfd_modem_port.c": [(0x56444652, 4)]}

HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "launchcore/persist.h"
#include "launchcore/storage.h"
static uint8_t disk[PERSIST_SIZE];
static unsigned erases, programs;
static const launchcore_storage_layout_t layout={.persistent_data_base=0,.persistent_data_size=PERSIST_SIZE,
 .persistent_data_erase_size=ERASE_SIZE,.persistent_data_write_size=WRITE_SIZE};
static launchcore_storage_status_t media_init(void){return LAUNCHCORE_STORAGE_OK;}
static const launchcore_storage_layout_t*media_layout(void){return &layout;}
static launchcore_storage_status_t media_read(uint32_t a,void*p,uint32_t n){assert(a<=PERSIST_SIZE&&n<=PERSIST_SIZE-a);memcpy(p,disk+a,n);return LAUNCHCORE_STORAGE_OK;}
static launchcore_storage_status_t media_erase(uint32_t a,uint32_t n){assert(a%ERASE_SIZE==0&&n%ERASE_SIZE==0&&a+n<=PERSIST_SIZE);memset(disk+a,0xFF,n);erases+=n/ERASE_SIZE;return LAUNCHCORE_STORAGE_OK;}
static launchcore_storage_status_t media_write(uint32_t a,const void*p,uint32_t n){
 assert(a%WRITE_SIZE==0&&a<=PERSIST_SIZE&&n<=PERSIST_SIZE-a);const uint8_t*b=p;
 for(uint32_t i=0;i<n;i++){assert((disk[a+i]&b[i])==b[i]);disk[a+i]=b[i];}programs++;return LAUNCHCORE_STORAGE_OK;}
const launchcore_storage_driver_t launchcore_board_storage_driver={.init=media_init,.read=media_read,.erase=media_erase,.write=media_write,.layout=media_layout};
/* The board's persistent_store boundary over the strict journal. */
launchcore_persist_status_t persistent_store_init(void){return LAUNCHCORE_PERSIST_OK;}
launchcore_persist_status_t persistent_store_get(uint32_t k,void*v,size_t*n){launchcore_persist_strict_info_t i;return launchcore_persist_get_strict(k,v,n,&i);}
launchcore_persist_status_t persistent_store_set(uint32_t k,const void*v,size_t n){launchcore_persist_strict_info_t i;return launchcore_persist_set_strict(k,v,n,&i);}
static const struct{uint32_t key;size_t size;}records[]={RECORDS};
static void fill(uint8_t*b,size_t n,uint32_t key,unsigned salt){for(size_t i=0;i<n;i++)b[i]=(uint8_t)(key>>(8*(i%4)))^(uint8_t)(i*31U+salt);}
static void healthy(unsigned salt,uint8_t fstate){
 launchcore_persist_strict_info_t info;assert(launchcore_persist_probe_strict(&info)==LAUNCHCORE_PERSIST_OK&&info.health==LAUNCHCORE_PERSIST_HEALTH_HEALTHY);
 for(size_t r=0;r<sizeof records/sizeof records[0];r++){uint8_t want[64],got[64];size_t n=sizeof got;
  fill(want,records[r].size,records[r].key,r==1?salt:0);
  assert(launchcore_persist_get_strict(records[r].key,got,&n,&info)==LAUNCHCORE_PERSIST_OK&&n==records[r].size&&!memcmp(want,got,n));}
 uint8_t s=0;size_t n=1;assert(launchcore_persist_get_strict(0x46535445U,&s,&n,&info)==LAUNCHCORE_PERSIST_OK&&n==1&&s==fstate);
}
#ifdef UNDERGLOW
UNDERGLOW_SOURCE
static uint8_t underglow_toggles(unsigned count,unsigned seed){
 SedsRouter router=0;assert(av_bay_underglow_init(&router)==SEDS_OK&&underglow_callback);
 uint32_t x=seed;uint8_t value=0;
 for(unsigned i=0;i<count;i++){
  x=x*1103515245U+12345U;
  /* Mostly bench clicking (10-500 ms apart), sometimes a pause long enough to settle. */
  tick+=(x>>16)%8U==0U?(x>>8)%9000U:10U+(x>>4)%490U;
  assert(av_bay_underglow_poll(&router)==SEDS_OK); /* the telemetry loop runs through the pause */
  value=(uint8_t)!value;SedsPacketView p={SEDS_DT_AV_BAY_UNDERGLOW,&value,1};
  const unsigned before=programs;assert(underglow_callback(&p,NULL)==SEDS_OK&&programs==before); /* never writes from the toggle */
  assert(av_bay_underglow_poll(&router)==SEDS_OK);
 }
 tick+=UNDERGLOW_PERSIST_MIN_INTERVAL_MS+UNDERGLOW_PERSIST_SETTLE_MS;assert(av_bay_underglow_poll(&router)==SEDS_OK);
 return value;
}
#endif
int main(void){
 launchcore_storage_set_driver(&launchcore_board_storage_driver);memset(disk,0xFF,sizeof disk);
 launchcore_persist_strict_info_t info;uint8_t buffer[64];
 fill(buffer,records[0].size,records[0].key,0);assert(launchcore_persist_provision_virgin_strict(records[0].key,buffer,records[0].size,&info)==LAUNCHCORE_PERSIST_OK);
 for(size_t r=1;r<sizeof records/sizeof records[0];r++){fill(buffer,records[r].size,records[r].key,0);assert(launchcore_persist_set_strict(records[r].key,buffer,records[r].size,&info)==LAUNCHCORE_PERSIST_OK);}
 /* Flight-state churn: every network value, far beyond one bank of records. */
 static const uint8_t states[]={1,0,1,2,6,2,1,7,16,13,15,0,1};uint8_t fstate=0;unsigned salt=0;
 for(unsigned i=0;i<FSTATE_CHANGES;i++){
  fstate=states[i%sizeof states];assert(persistent_store_set(0x46535445U,&fstate,1)==LAUNCHCORE_PERSIST_OK);
  if(i%977U==0U){salt=i;fill(buffer,records[1].size,records[1].key,salt);assert(launchcore_persist_set_strict(records[1].key,buffer,records[1].size,&info)==LAUNCHCORE_PERSIST_OK);}
  if(i%500U==0U)healthy(salt,fstate);
 }
 healthy(salt,fstate);
 const unsigned fstate_erases=erases;
#ifdef UNDERGLOW
 const uint32_t start=tick;const uint8_t last=underglow_toggles(UNDERGLOW_TOGGLES,0x5EED5U);
 const uint32_t minutes=(tick-start)/60000U;
 printf("underglow: %u toggles over %u min -> %u writes\n",UNDERGLOW_TOGGLES,(unsigned)minutes,(unsigned)g_av_bay_underglow_persist_writes);
 assert(g_av_bay_underglow_persist_errors==0&&g_av_bay_underglow_persist_writes>=1U&&g_av_bay_underglow_persist_writes<=minutes+2U);
 uint8_t stored=9;size_t n=1;assert(launchcore_persist_get_strict(0x55474C57U,&stored,&n,&info)==LAUNCHCORE_PERSIST_OK&&n==1&&stored==last);
 healthy(salt,fstate);
#endif
 printf("store healthy after %u flight-state changes; %u page erases (%u from flight state)\n",FSTATE_CHANGES,erases,fstate_erases);
 return 0;
}
'''

UNDERGLOW_STUBS = r'''
typedef int SedsResult;typedef int SedsRouter;
typedef struct{uint32_t ty;const uint8_t*payload;size_t payload_len;}SedsPacketView;
#define SEDS_OK 0
#define SEDS_HANDLER_ERROR 1
#define SEDS_BAD_ARG 2
#define SEDS_DT_AV_BAY_UNDERGLOW 7
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define BLUE_LED_GPIO_Port 0
#define BLUE_LED_Pin 0
#define BLUE_LEDS_GPIO_Port 0
#define BLUE_LEDS_Pin 0
volatile uint32_t g_telemetry_discovery_seen=1;
static uint32_t tick;
static uint32_t HAL_GetTick(void){return tick;}
static void HAL_GPIO_WritePin(int port,int pin,int value){(void)port;(void)pin;(void)value;}
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __set_PRIMASK(uint32_t m){(void)m;}
static SedsResult(*underglow_callback)(const SedsPacketView*,void*);
static int seds_router_enable_network_variable(SedsRouter*r,int t,bool a,bool b){(void)r;(void)t;(void)a;(void)b;return 0;}
static int seds_router_on_network_variable_update(SedsRouter*r,int t,SedsResult(*f)(const SedsPacketView*,void*),void*u){(void)r;(void)t;(void)u;underglow_callback=f;return 0;}
static int seds_router_request_managed_variable(SedsRouter*r,int t){(void)r;(void)t;return 0;}
'''


@unittest.skipUnless(shutil.which("cc"), "native C compiler unavailable")
class PersistEnduranceTests(unittest.TestCase):
    def test_setting_churn_leaves_the_store_healthy(self):
        lc = launchcore_source()
        records = RECORDS + [record for module, extra in EXTRA.items()
                             if (ROOT / "Core/Src" / module).exists() for record in extra]
        defines = ["-DPERSIST_SIZE=%du" % board_value("PERSIST_SIZE"),
                   "-DERASE_SIZE=%du" % board_value("FLASH_ERASE_SIZE"),
                   "-DWRITE_SIZE=%du" % write_size(), "-DFSTATE_CHANGES=%du" % FSTATE_CHANGES,
                   "-DRECORDS=" + ",".join("{0x%08Xu,%d}" % record for record in records)]
        code = HARNESS
        if UNDERGLOW.exists():
            source = "\n".join(line for line in UNDERGLOW.read_text().splitlines()
                               if not line.startswith('#include "'))
            code = code.replace("UNDERGLOW_SOURCE", UNDERGLOW_STUBS + source)
            defines += ["-DUNDERGLOW", "-DTELEMETRY_ENABLED=1", "-DUNDERGLOW_TOGGLES=10000u"]
        with tempfile.TemporaryDirectory(prefix="persist-endurance-") as directory:
            work = Path(directory)
            (work / "endurance.c").write_text(code)
            exe = work / "endurance"
            subprocess.run(["cc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                            "-fsanitize=undefined", *defines, "-I", str(lc / "bootloader/include"),
                            str(work / "endurance.c"), str(ROOT / "Bootloader/storage_dispatch.c"),
                            str(lc / "bootloader/src/persist.c"), str(lc / "bootloader/src/crc32.c"),
                            "-o", str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=300)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            print(result.stdout.strip())


if __name__ == "__main__":
    unittest.main()
