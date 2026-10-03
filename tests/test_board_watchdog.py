from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class BoardWatchdogTests(unittest.TestCase):
    def test_only_complete_fresh_task_progress_feeds_watchdog(self):
        src=(ROOT/'Core/Src/board_watchdog.c').read_text()
        src='\n'.join(line for line in src.splitlines() if not line.startswith('#include'))
        stub=r'''
#include <stdint.h>
#include <assert.h>
typedef uint32_t ULONG;
#define BOARD_WATCHDOG_ENABLE 1
#define BOARD_WATCHDOG_REQUIRED_MASK 3U
#define TELEMETRY_ENABLED 1
#define __HAL_RCC_CLEAR_RESET_FLAGS() ((void)0)
#define TX_TIMER_TICKS_PER_SECOND 1000U
static struct {uint32_t CSR,RSR;} rcc;
#define RCC (&rcc)
static struct {uint32_t KR,PR,RLR,SR;} iwdg;
#define IWDG (&iwdg)
static uint32_t tick,mask;
static ULONG tx_time_get(void){return tick;}
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t m){mask=m;}
static void __NOP(void){}
'''
        main=r'''
int main(void){
 rcc.CSR=0x123;board_watchdog_start();
 assert(g_watchdog_reset_flags==0x123 && g_watchdog_started==1);
 assert(iwdg.PR==6 && iwdg.RLR==2047);
 tick=250;board_watchdog_progress(1);assert(g_watchdog_feed_count==0);
 board_watchdog_progress(2);assert(g_watchdog_feed_count==1 && !mask);
 for(unsigned i=0;i<100;i++){tick+=250;board_watchdog_progress(2);}
 assert(g_watchdog_feed_count==1 && g_watchdog_missing_mask==1);
 board_watchdog_progress(1);assert(g_watchdog_feed_count==2);
 board_watchdog_progress(1);board_watchdog_progress(2);
 assert(g_watchdog_feed_count==2); /* frozen scheduler clock never feeds */
 tick=UINT32_MAX-100;board_watchdog_progress(1);
 unsigned fed=g_watchdog_feed_count;
 tick=200;board_watchdog_progress(1);board_watchdog_progress(2);
 assert(g_watchdog_feed_count==fed+1); /* tick wrap */
}
'''
        with tempfile.TemporaryDirectory() as d:
            exe=str(Path(d)/'watchdog')
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',exe],input=stub+src+main,text=True,check=True)
            subprocess.run([exe],check=True,timeout=5)

    def test_bootloader_feeds_already_running_watchdog(self):
        src=(ROOT/'Bootloader/platform.c').read_text()
        self.assertIn('void platform_feed_watchdog(void) { IWDG->KR = 0xAAAAU; }',src)
        self.assertRegex(src,r'void platform_early_init\(void\)\s*\{\s*platform_feed_watchdog\(\);')
        self.assertRegex(src,r'void platform_deinit_before_jump\(void\)\s*\{\s*platform_feed_watchdog\(\);')
