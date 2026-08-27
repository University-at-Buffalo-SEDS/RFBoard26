import tempfile
import unittest
from pathlib import Path
from unittest import mock

import build


class OtaBuildScriptTests(unittest.TestCase):
    def test_ota_shortcut_is_available(self):
        args = build.make_parser().parse_args(["build", "--ota"])
        self.assertTrue(args.ota)

    def test_bsp_ota_layout_detection(self):
        cases = {
            "delta-macro": ("delta", "#define BOARD_DELTA_SIZE 0x6000u\n", "", 0x6000),
            "ab": ("ab", "#define BOARD_SLOT_B_SIZE 0x40000u\n", "", 0x40000),
            "delta-slot-b": (
                "delta",
                "#define BOARD_SLOT_B_SIZE 0x8000u\n",
                "static const int layout = {.slot_b_is_delta = true};\n",
                0x8000,
            ),
            "staging": ("staging", "#define BOARD_APP_STAGING_SIZE 0x50000u\n", "", 0x50000),
            "recovery": ("recovery", "#define BOARD_SLOT_A_SIZE 0x70000u\n", "", 0),
        }
        for label, (expected, config, storage, expected_size) in cases.items():
            with self.subTest(label), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                (root / "Bootloader").mkdir()
                (root / "Bootloader" / "board_config.h").write_text(
                    config, encoding="utf-8"
                )
                (root / "Bootloader" / "storage_internal_flash.c").write_text(
                    storage, encoding="utf-8"
                )
                layout = build.detect_ota_layout(root)
                self.assertEqual(layout.mode, expected)
                self.assertEqual(layout.secondary_size, expected_size)

    def test_first_ota_build_is_full_recovery_seds(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "Bootloader").mkdir()
            (root / "Bootloader" / "board_config.h").write_text(
                "#define TEST_SLOT_A_BASE 0x08004000u\n", encoding="utf-8"
            )
            cfg_args = dict(
                repo_root=root,
                build_type="Release",
                telemetry=True,
                generator="Ninja",
                toolchain_file=root / "toolchain.cmake",
                build_subdir="Release",
                project_name="TestBoard",
                artifact=None,
            )
            if "use_preset" in build.BuildConfig.__dataclass_fields__:
                cfg_args["use_preset"] = False
            cfg = build.BuildConfig(**cfg_args)

            def fake_build(_ui, selected_cfg, _target):
                selected_cfg.build_dir.mkdir(parents=True, exist_ok=True)
                package = selected_cfg.build_dir / "TestBoard.launchcore.img"
                package.write_bytes(b"packaged-launchcore-image")
                return selected_cfg.build_dir / "TestBoard.elf", selected_cfg.build_dir / "TestBoard.bin"

            ui = mock.Mock()
            with mock.patch.object(build, "configure_and_build", side_effect=fake_build):
                artifact = build.build_selected_artifact(
                    ui, cfg, "ota", None, None, False
                )

            self.assertEqual(artifact.kind, "ota-recovery")
            self.assertEqual(artifact.path.suffix, ".seds")
            self.assertEqual(artifact.path.read_bytes(), b"packaged-launchcore-image")


if __name__ == "__main__":
    unittest.main()
