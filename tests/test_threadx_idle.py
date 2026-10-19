"""Compile the scheduler's actual preprocessor path for the board configuration."""
from pathlib import Path
import subprocess
import unittest
ROOT = Path(__file__).resolve().parents[1]

class ThreadXIdleTests(unittest.TestCase):
    def test_idle_uses_interrupt_wait_without_changing_timer_or_clock_setup(self):
        schedules=list((ROOT/'Middlewares/ST/threadx/ports').glob('*/gnu/src/tx_thread_schedule.S'))
        self.assertEqual(len(schedules),1)
        self.assertIn('target_compile_definitions(ThreadX PRIVATE TX_ENABLE_WFI)', (ROOT/'CMakeLists.txt').read_text())
        out=subprocess.check_output(['cc','-E','-P','-x','assembler-with-cpp',
            '-DTX_INCLUDE_USER_DEFINE_FILE','-DTX_ENABLE_WFI','-I',str(ROOT/'Core/Inc'),str(schedules[0])],text=True)
        idle=out.split('__tx_ts_wait:',1)[1].split('__tx_ts_ready:',1)[0]
        self.assertIn('CBNZ',idle)
        self.assertIn('DSB',idle)
        self.assertIn('WFI',idle)
        self.assertIn('ISB',idle)
        self.assertLess(idle.index('CBNZ'),idle.index('WFI'))
        self.assertLess(idle.index('DSB'),idle.index('WFI'))
        self.assertLess(idle.index('WFI'),idle.index('ISB'))
        self.assertNotIn('BL tx_low_power_enter',idle)
        self.assertNotIn('BL tx_low_power_exit',idle)
        self.assertNotIn('STOP',idle)
