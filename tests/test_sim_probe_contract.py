import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
from sim import run_full

class SimulatorProbeContractTests(unittest.TestCase):
    def layout(self, root):
        (root/'sim').mkdir()
        (root/'build/Test').mkdir(parents=True)
        (root/'build/Test/board.elf').touch()
        (root/'sim/board.json').write_text(json.dumps({'artifacts':{'elf':'build/Release/board.elf'},'execution':{'memory_probes':[{'name':'allocation_failures','symbol':'g_alloc_fail','maximum':0}]}}))

    def test_gateway_qualification_uses_pico_baud_without_changing_radio(self):
        with mock.patch.dict(run_full.os.environ, {}, clear=True):
            gateway = run_full.network_firmware_build_command(Path('/gateway'), 'gateway-board26', True)
            self.assertEqual(gateway[-2:], ['--pico-baud', '1000000'])
            radio = run_full.network_firmware_build_command(Path('/rf'), 'RFBoard26', True)
            self.assertNotIn('--pico-baud', radio)
        with mock.patch.dict(run_full.os.environ, {'SEDS_FIRMWARE_SIM_PICO_BAUD':'115200'}):
            self.assertEqual(run_full.network_firmware_build_command(Path('/g'), 'gateway-board', True)[-1], '115200')
        with mock.patch.dict(run_full.os.environ, {'SEDS_FIRMWARE_SIM_PICO_BAUD':'0'}):
            with self.assertRaises(RuntimeError):
                run_full.network_firmware_build_command(Path('/g'), 'gateway-board26', True)

    def test_missing_failure_counter_fails_before_starting_simulator(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);self.layout(root)
            with mock.patch.object(run_full.shutil,'which',return_value='/tools/arm-none-eabi-nm'), mock.patch.object(run_full.subprocess,'check_output',return_value='08000000 T Reset_Handler\n'):
                with self.assertRaisesRegex(RuntimeError,'g_alloc_fail'):
                    run_full.load_layout_for_build(root,'Test')

    def test_required_counter_is_preserved_in_loaded_layout(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);self.layout(root)
            with mock.patch.object(run_full.shutil,'which',return_value='/tools/arm-none-eabi-nm'), mock.patch.object(run_full.subprocess,'check_output',return_value='20000000 B g_alloc_fail\n'):
                layout=run_full.load_layout_for_build(root,'Test')
            self.assertEqual(layout['execution']['memory_probes'][0]['maximum'],0)
            self.assertEqual(layout['artifacts']['elf'],'build/Test/board.elf')

    def test_cmake_toolchain_nm_is_used_when_not_on_path(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);self.layout(root)
            nm=root/'compiler-nm';nm.touch()
            (root/'build/Test/CMakeCache.txt').write_text(f'CMAKE_NM:FILEPATH={nm}\n')
            with mock.patch.object(run_full.shutil,'which',return_value=None), mock.patch.object(run_full.subprocess,'check_output',return_value='20000000 B g_alloc_fail\n') as invoke:
                run_full.load_layout_for_build(root,'Test')
            self.assertEqual(invoke.call_args.args[0][0],str(nm))

if __name__=='__main__':unittest.main()
