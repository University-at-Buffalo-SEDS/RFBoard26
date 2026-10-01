import tempfile
import unittest
from pathlib import Path
from unittest import mock
import build

class BuildAllocatorTests(unittest.TestCase):
    def test_allocator_cli_and_config(self):
        for command in ('build', 'flash', 'test'):
            for allocator in ('threadx', 'tlsf'):
                args=build.make_parser().parse_args([command,'--release','--allocator',allocator])
                cfg=build.build_cfg_from_args(mock.Mock(),args)
                self.assertEqual(cfg.allocator,allocator)
            args=build.make_parser().parse_args([command,'--release'])
            self.assertEqual(build.build_cfg_from_args(mock.Mock(),args).allocator,'threadx')

    def test_all_configure_paths_explicitly_override_cached_allocator(self):
        preset_modes=(False,True) if 'use_preset' in build.BuildConfig.__dataclass_fields__ else (False,)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for preset in preset_modes:
                for allocator,flag in [('tlsf','ON'),('threadx','OFF')]:
                    fields=dict(repo_root=root,build_type='Release',telemetry=True,
                                generator='Ninja',toolchain_file=root/'toolchain.cmake',
                                build_subdir='Release',project_name='test',artifact=None,
                                allocator=allocator)
                    if 'use_preset' in build.BuildConfig.__dataclass_fields__: fields['use_preset']=preset
                    cfg=build.BuildConfig(**fields)
                    cfg.build_dir.mkdir(parents=True,exist_ok=True)
                    # Mock just process execution/artifact discovery: exercise the real
                    # configure command generation, including a stale opposite cache.
                    (cfg.build_dir/'CMakeCache.txt').write_text(
                        'CMAKE_C_COMPILER:FILEPATH=/usr/bin/arm-none-eabi-gcc\n'
                        f'TELEMETRY_USE_TLSF:BOOL={"OFF" if flag=="ON" else "ON"}\n')
                    with mock.patch.object(build,'run') as run, mock.patch.object(build,'pick_elf',return_value=cfg.build_dir/'test.elf'):
                        build.configure_and_build(mock.Mock(),cfg)
                    command=run.call_args_list[0].args[1]
                    self.assertEqual(command.count(f'-DTELEMETRY_USE_TLSF={flag}'),1)
                    self.assertEqual('--preset' in command,preset)
