import contextlib
import io
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import build


class OtaBuildScriptTests(unittest.TestCase):
    def test_all_tests_preserve_the_selected_build_mode(self):
        release = build.make_parser().parse_args(["test", "--all", "--release"])
        debug = build.make_parser().parse_args(["test", "--all"])
        legacy = build.make_parser().parse_args(["test", "--full"])

        self.assertTrue(release.all_tests)
        self.assertTrue(release.release)
        self.assertTrue(debug.all_tests)
        self.assertFalse(debug.release)
        self.assertTrue(legacy.all_tests)

    def test_simulation_layout_uses_selected_build_directory(self):
        from sim.run_full import load_layout_for_build

        root = Path(build.__file__).resolve().parent
        layout = load_layout_for_build(root, "Selected_Test_Build")
        for artifact in layout["artifacts"].values():
            self.assertEqual(Path(artifact).parts[1], "Selected_Test_Build")

    def test_all_tests_report_an_unavailable_docker_daemon(self):
        from sim import run_full

        probe = mock.Mock(returncode=1, stdout="", stderr="daemon unavailable")
        with mock.patch.object(run_full.shutil, "which", return_value="/usr/bin/docker"):
            with mock.patch.object(run_full.subprocess, "run", return_value=probe):
                with self.assertRaisesRegex(RuntimeError, "daemon is not available"):
                    run_full.require_docker()

    def test_missing_registry_image_is_built_from_a_fresh_clone(self):
        from sim import run_full

        missing = mock.Mock(returncode=1, stdout="", stderr="denied")
        success = mock.Mock(returncode=0, stdout="", stderr="")
        ui = mock.Mock()
        with mock.patch.dict(run_full.os.environ, {}, clear=True):
            with mock.patch.object(run_full.shutil, "which", return_value="/usr/bin/git"):
                with mock.patch.object(
                    run_full.subprocess,
                    "run",
                    side_effect=[missing, missing, missing, success, success],
                ) as execute:
                    image = run_full.resolve_simulator_image(
                        ui, "/usr/bin/docker", Path("/board"), "stm32g4"
                    )

        self.assertEqual(image, "seds-firmware-simulator:stm32g4-local")
        commands = [call.args[0] for call in execute.call_args_list]
        self.assertTrue(any("clone" in command for command in commands))

    def test_flash_defaults_to_combined_factory_image(self):
        args = build.make_parser().parse_args(["flash", "--release", "--method", "dfu"])
        self.assertEqual(args.image, "factory")
        self.assertFalse(args.app_only)

    def test_dfu_flash_leaves_rom_bootloader_after_download(self):
        ui = mock.Mock()
        image = Path("RFBoard26.factory.bin")
        dfu_output = ("Download [=========================] 100%\r"
                      "File downloaded successfully\nSubmitting leave request...\n"
                      "dfu-util: Error during download get_status\n")
        process = mock.Mock(stdout=io.BytesIO(dfu_output.encode("utf-8")))
        process.wait.return_value = 74
        visible_output = io.StringIO()
        with mock.patch.object(build, "which", return_value="/usr/local/bin/dfu-util"), \
             mock.patch.object(build.subprocess, "Popen", return_value=process) as popen:
            with contextlib.redirect_stdout(visible_output):
                build.flash_dfu(ui, image, "0x08000000")
        command = popen.call_args.args[0]
        self.assertEqual(command[4], "0x08000000:leave")
        self.assertEqual(
            visible_output.getvalue(),
            dfu_output.replace("dfu-util: Error during download get_status\n", ""),
        )

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
