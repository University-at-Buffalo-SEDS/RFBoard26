import subprocess
import unittest
from unittest import mock
import build

class FlashTargetTests(unittest.TestCase):
    def test_wrong_or_missing_device_id_refuses_flash(self):
        for output in ('Device ID : 0x999', '', 'Device ID : 0x479'):
            if output.endswith('0x479'):
                expected = build.verify_stm32_target.__code__.co_consts
                if 0x479 in expected: continue
            with mock.patch.object(build.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, output, '')):
                with self.assertRaisesRegex(RuntimeError, 'Refusing flash'):
                    build.verify_stm32_target('programmer', 'port=SWD')

    def test_expected_device_and_successful_connection(self):
        expected=next(n for n in build.verify_stm32_target.__code__.co_consts if isinstance(n,int) and n in (0x478,0x479,0x482))
        for status in (0,1):
            with mock.patch.object(build.subprocess,'run',return_value=subprocess.CompletedProcess([],status,f'\x1b[90mDevice ID : {expected:#x}\x1b[0m','')) as run:
                if status:
                    with self.assertRaises(RuntimeError):build.verify_stm32_target('programmer','port=SWD mode=UR')
                else:build.verify_stm32_target('programmer','port=SWD mode=UR')
                self.assertEqual(run.call_args.args[0],['programmer','-c','port=SWD','mode=UR'])
