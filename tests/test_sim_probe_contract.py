import json
import subprocess
import sys
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

    def test_clean_network_build_produces_layout_factory_for_every_board(self):
        # The real supported entrypoints distinguish application-only builds
        # from factory builds. Exercise that interface from empty artifact dirs,
        # rather than allowing a previous build's factory file to mask a miss.
        boards = ['FlightComputer26', 'RFBoard26', 'PowerBoard26',
                  'gateway-board26', 'ActuatorBoard26', 'ValveBoard26', 'DAQ-Board']
        with tempfile.TemporaryDirectory() as directory:
            for repository in boards:
                with self.subTest(repository=repository):
                    root = Path(directory) / repository
                    (root / 'sim').mkdir(parents=True)
                    subdir = 'Release' if repository in {'FlightComputer26', 'RFBoard26'} else 'Release_Script'
                    artifacts = {name: f'build/{subdir}/board.{suffix}' for name, suffix in
                                 [('elf', 'elf'), ('firmware', 'launchcore.img'), ('factory', 'factory.bin')]}
                    (root / 'sim/board.json').write_text(json.dumps({'artifacts': artifacts}))
                    (root / 'build.py').write_text("""import json,sys
from pathlib import Path
root=Path(__file__).parent
args=sys.argv[1:]
kind=args[args.index('--image')+1] if '--image' in args else args[-1]
for name,value in json.loads((root/'sim/board.json').read_text())['artifacts'].items():
    if name=='factory' and kind!='factory':continue
    path=root/value;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(name.encode())
""")
                    command = run_full.network_firmware_build_command(root, repository, True)
                    self.assertEqual(command[0], sys.executable)
                    subprocess.run(command, cwd=root, check=True)
                    layout = run_full.load_layout_for_build(root, None)
                    for name in ['elf', 'firmware', 'factory']:
                        self.assertTrue((root / layout['artifacts'][name]).is_file(), name)
                    self.assertFalse((root / 'build/Debug').exists())

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
