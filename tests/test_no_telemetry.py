"""The offline CLI preserves local boot state without SDK headers or handlers."""
import dataclasses
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest import mock
import build

ROOT = Path(__file__).resolve().parents[1]

class NoTelemetryBuildTests(unittest.TestCase):
    def test_actual_cli_passes_telemetry_off_to_factory_configuration(self):
        args=build.make_parser().parse_args(["--build-subdir","OfflineTest","build","--release","--no-telemetry"])
        cfg=build.build_cfg_from_args(mock.Mock(),args)
        self.assertFalse(cfg.telemetry)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            cfg=dataclasses.replace(cfg,repo_root=root,project_name="TestBoard")
            elf=cfg.build_dir/"TestBoard.elf"
            commands=[]
            with mock.patch.object(build,"run",side_effect=lambda ui,cmd,**kwargs:commands.append(cmd)),mock.patch.object(build,"pick_elf",return_value=elf),mock.patch.object(build,"assert_embedded_cache",create=True):
                build.configure_and_build(mock.Mock(),cfg,"factory-image")
            self.assertIn("-DENABLE_TELEMETRY=OFF",commands[0])
            self.assertIn("factory-image",commands[1])

    def test_local_flash_restore_needs_no_sdk_or_network_calls(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/"launchcore").mkdir()
            (root/"launchcore/persist.h").write_text("typedef enum {LAUNCHCORE_PERSIST_OK=0, LAUNCHCORE_PERSIST_NOT_FOUND=1,LAUNCHCORE_PERSIST_STORAGE_ERROR=2} launchcore_persist_status_t;\n")
            (root/"main.h").write_text("#include <stdint.h>\n#define BLUE_LEDS_GPIO_Port ((void*)1)\n#define BLUE_LEDS_Pin 1\n#define BLUE_LED_GPIO_Port BLUE_LEDS_GPIO_Port\n#define BLUE_LED_Pin BLUE_LEDS_Pin\n#define GPIO_PIN_SET 1\n#define GPIO_PIN_RESET 0\nvoid HAL_GPIO_WritePin(void*,uint16_t,int);\n")
            (root/"restore.c").write_text(r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include "flight_state_cache.h"
#include "av_bay_underglow.h"
#include "persistent_store.h"
static int inits,drives;
launchcore_persist_status_t persistent_store_init(void){++inits;return LAUNCHCORE_PERSIST_OK;}
launchcore_persist_status_t persistent_store_get(uint32_t key,void*value,size_t*size){assert(*size==1);*(uint8_t*)value=key==0x46535445u?3:1;return LAUNCHCORE_PERSIST_OK;}
launchcore_persist_status_t persistent_store_set(uint32_t key,const void*value,size_t size){(void)key;(void)value;(void)size;assert(0);return LAUNCHCORE_PERSIST_STORAGE_ERROR;}
void HAL_GPIO_WritePin(void*port,uint16_t pin,int state){assert(port==(void*)1&&pin==1&&state==1);++drives;}
int main(void){flight_state_cache_restore();flight_state_cache_restore();assert(flight_state_cache_has_value());assert(flight_state_cache_value()==3);av_bay_underglow_restore();av_bay_underglow_restore();assert(inits==2&&drives==1);return 0;}
''')
            executable=root/"restore"
            subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror","-Wno-unknown-attributes","-I"+str(root),"-I"+str(ROOT/"Core/Inc"),str(ROOT/"Core/Src/flight_state_cache.c"),str(ROOT/"Core/Src/av_bay_underglow.c"),str(root/"restore.c"),"-o",str(executable)],check=True)
            subprocess.run([str(executable)],check=True)

    def test_watchdog_without_sdk_requires_real_acquisition_progress(self):
        source=(ROOT/'Core/Src/board_watchdog.c').read_text()
        source='\n'.join(line for line in source.splitlines() if not line.startswith('#include'))
        stub=r'''
#include <stdint.h>
#include <assert.h>
#include "board_uptime.h"
#include "board_watchdog.h"
typedef uint32_t ULONG;
#define BOARD_WATCHDOG_ENABLE 1
#define IWDG_SR_PVU 1U
#define IWDG_SR_RVU 2U
#define IWDG_SR_WVU 4U
#define IWDG_SR_EWU 8U
#define __HAL_RCC_CLEAR_RESET_FLAGS() ((void)0)
#define TX_TIMER_TICKS_PER_SECOND 1000U
static struct {uint32_t CSR,RSR;} rcc;
#define RCC (&rcc)
static struct {uint32_t KR,PR,RLR,SR;} iwdg;
#define IWDG (&iwdg)
static uint32_t tick,mask,init_tick;
static ULONG tx_time_get(void){return tick;}
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t value){mask=value;}
static uint32_t HAL_GetTick(void){return init_tick++;}
static void __NOP(void){}
'''
        main=r'''
int main(void){
 board_watchdog_start();assert(g_watchdog_started==1);
 assert(g_watchdog_missing_mask==BOARD_WATCHDOG_ACQUISITION);
 tick=250;board_watchdog_progress(BOARD_WATCHDOG_NETWORK);
 assert(g_watchdog_feed_count==0);
 board_watchdog_progress(BOARD_WATCHDOG_ACQUISITION);
 assert(g_watchdog_feed_count==1);
 for(unsigned n=0;n<100;n++){tick+=250;board_watchdog_progress(BOARD_WATCHDOG_NETWORK);}
 assert(g_watchdog_feed_count==1 && g_watchdog_missing_mask==BOARD_WATCHDOG_ACQUISITION);
 board_watchdog_progress(BOARD_WATCHDOG_ACQUISITION);
 assert(g_watchdog_feed_count==2 && mask==0);
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            executable=Path(directory)/'watchdog'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(executable)],input=stub+source+main,text=True,check=True)
            subprocess.run([str(executable)],check=True,timeout=5)
