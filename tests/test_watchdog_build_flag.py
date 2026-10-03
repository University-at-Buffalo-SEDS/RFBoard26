import tempfile
import unittest
from pathlib import Path
from unittest import mock
import build
from dataclasses import fields

class WatchdogBuildFlagTests(unittest.TestCase):
    def test_watchdog_flag_reaches_cmake_and_default_turns_it_off(self):
        for enabled in (False, True):
            args = build.make_parser().parse_args(["build"] + (["--watchdog"] if enabled else []))
            self.assertEqual(args.watchdog, enabled)
            with tempfile.TemporaryDirectory() as directory:
                values = dict(repo_root=Path(directory), build_type="Release", telemetry=True,
                              generator="Ninja", toolchain_file=Path("toolchain.cmake"),
                              build_subdir="Release", project_name="Board", artifact="Board",
                              use_preset=False, watchdog=enabled)
                cfg = build.BuildConfig(**{f.name: values[f.name] for f in fields(build.BuildConfig) if f.name in values})
                cfg.build_dir.mkdir(parents=True)
                (cfg.build_dir / "Board.elf").touch()
                commands = []
                with mock.patch.object(build, "run", side_effect=lambda ui, cmd, **kwargs: commands.append(cmd)):
                    build.configure_and_build(mock.Mock(), cfg)
                expected = "-DENABLE_BOARD_WATCHDOG=" + ("ON" if enabled else "OFF")
                self.assertIn(expected, commands[0])
