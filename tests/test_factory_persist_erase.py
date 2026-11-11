"""A factory flash erases the board's LaunchCore persistent store by default.

Factory flash does not erase the store region on its own, so records from an
older flash layout survive into the new image (bench, 2026-11-10: a stale LCMD
record blocked compaction and the board refused every command). The erase is
an explicit option: on by default for factory images, off for app-only, and
--keep-persist / --erase-persist override it.
"""
import io
import re
import unittest
from pathlib import Path
from unittest import mock

import build

ROOT = Path(__file__).resolve().parents[1]


def board_value(suffix):
    text = (ROOT / "Bootloader/board_config.h").read_text()
    match = re.search(r"(?m)^#define\s+\w*" + suffix + r"\s+(0x[0-9A-Fa-f]+|\d+)u?\s*$", text)
    return int(match.group(1), 0)


BASE = board_value("PERSIST_BASE")
SIZE = board_value("PERSIST_SIZE")
PAGE = board_value("FLASH_ERASE_SIZE")
FLASH = board_value("INTERNAL_FLASH_BASE")
PAGES = [str(page) for page in range((BASE - FLASH) // PAGE, (BASE - FLASH + SIZE) // PAGE)]


def flash_args(*extra):
    return build.make_parser().parse_args(["flash", "--release", *extra])


class FactoryPersistEraseTests(unittest.TestCase):
    def test_factory_flash_erases_the_store_by_default(self):
        self.assertTrue(build.persist_erase_selected(flash_args(), "factory"))
        self.assertFalse(build.persist_erase_selected(flash_args("--image", "firmware"), "firmware"))
        self.assertFalse(build.persist_erase_selected(flash_args("--keep-persist"), "factory"))
        self.assertTrue(build.persist_erase_selected(flash_args("--image", "firmware", "--erase-persist"), "firmware"))
        with mock.patch("sys.stderr", io.StringIO()), self.assertRaises(SystemExit):
            flash_args("--keep-persist", "--erase-persist")

    def test_region_is_the_board_store_on_whole_pages(self):
        region = build.persist_region(ROOT)
        self.assertEqual((region.base, region.size), (BASE, SIZE))
        self.assertEqual([str(page) for page in region.pages], PAGES)
        self.assertTrue(PAGES)
        self.assertIn(f"0x{BASE:08X}", build.describe_persist_erase(region))

    def test_cubeprogrammer_erases_store_pages_before_writing(self):
        ui = mock.Mock()
        region = build.persist_region(ROOT)
        with mock.patch.object(build, "verify_stm32_target"), \
                mock.patch.object(build, "run") as run:
            build.flash_stm32prog_cli(ui, Path("Board.factory.bin"), "0x08000000", True,
                                      "/opt/STM32_Programmer_CLI", "port=SWD", [], erase=region)
        command = run.call_args.args[1]
        erase = command.index("-e")
        self.assertEqual(command[erase + 1:erase + 1 + len(PAGES)], PAGES)
        self.assertLess(erase, command.index("-w"))

    def test_st_flash_erases_store_range_then_writes(self):
        ui = mock.Mock()
        region = build.persist_region(ROOT)
        version = mock.Mock(stdout="v1.8.0\n", stderr="", returncode=0)
        with mock.patch.object(build, "which", return_value="/usr/bin/st-flash"), \
                mock.patch.object(build.subprocess, "run", return_value=version), \
                mock.patch.object(build, "run") as run:
            build.flash_st_flash(ui, Path("Board.factory.bin"), "0x08000000", True, erase=region)
        erase, write = [call.args[1] for call in run.call_args_list]
        self.assertEqual(erase, ["st-flash", "erase", f"0x{BASE:08x}", str(SIZE)])
        self.assertEqual(write[-3:], ["write", "Board.factory.bin", "0x08000000"])

    def test_old_st_flash_cannot_erase_a_range(self):
        region = build.persist_region(ROOT)
        version = mock.Mock(stdout="v1.7.0\n", stderr="", returncode=0)
        with mock.patch.object(build, "which", return_value="/usr/bin/st-flash"), \
                mock.patch.object(build.subprocess, "run", return_value=version), \
                mock.patch.object(build, "run") as run, \
                self.assertRaises(build.FriendlyError):
            build.flash_st_flash(mock.Mock(), Path("Board.factory.bin"), "0x08000000", True, erase=region)
        run.assert_not_called()

    def test_dfu_factory_flash_mass_erases_first(self):
        region = build.persist_region(ROOT)
        process = mock.Mock(stdout=io.BytesIO(b"File downloaded successfully\n"))
        process.wait.return_value = 0
        with mock.patch.object(build, "which", return_value="/usr/bin/dfu-util"), \
                mock.patch.object(build.subprocess, "Popen", return_value=process) as popen, \
                mock.patch("sys.stdout", io.StringIO()):
            build.flash_dfu(mock.Mock(), Path("Board.factory.bin"), "0x08000000", erase=region)
        self.assertEqual(popen.call_args.args[0][4], "0x08000000:mass-erase:force:leave")
        with mock.patch.object(build, "which", return_value="/usr/bin/dfu-util"), \
                mock.patch.object(build.subprocess, "Popen") as popen, \
                self.assertRaises(build.FriendlyError):
            build.flash_dfu(mock.Mock(), Path("Board.bin"), "0x08004000", erase=region)
        popen.assert_not_called()

    def test_gdb_methods_refuse_a_persist_erase(self):
        args = flash_args("--method", "st-util")
        artifact = build.BuiltArtifact("factory", Path("Board.factory.bin"), "0x08000000", Path("Board.elf"))
        with mock.patch.object(build, "flash_st_util") as flash, \
                self.assertRaises(build.FriendlyError):
            build.flash_selected(mock.Mock(), ROOT, args, artifact)
        flash.assert_not_called()


if __name__ == "__main__":
    unittest.main()
