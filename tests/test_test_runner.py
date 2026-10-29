import contextlib
import io
import unittest
from pathlib import Path
from unittest import mock

import build


class TestRunnerReportingTests(unittest.TestCase):
    def test_success_summary_is_explicit(self):
        ui = mock.Mock()
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            build._print_test_summary(
                ui,
                [
                    ("Host and Python unit tests", "PASS"),
                    ("GoogleTest unit tests", "PASS"),
                ],
            )

        self.assertIn("Firmware test summary", output.getvalue())
        self.assertIn("GoogleTest unit tests", output.getvalue())
        ui.say.assert_called_once_with("ok", "All 2 test stages passed.")

    def test_failed_stage_has_actionable_help_and_non_success_result(self):
        ui = mock.Mock()
        results = []
        output = io.StringIO()

        def fail():
            raise RuntimeError("compiler rejected test source")

        with contextlib.redirect_stdout(output):
            with self.assertRaises(build.FriendlyError) as raised:
                build._run_test_stage(
                    ui, results, "GoogleTest unit tests", fail
                )

        self.assertEqual(results, [("GoogleTest unit tests", "FAIL")])
        self.assertIn("Possible solutions", str(raised.exception))
        self.assertIn("C++17 compiler", str(raised.exception))
        self.assertIn("FAIL", output.getvalue())

    def test_network_peer_builds_include_factory_boot_images(self):
        from sim.run_full import network_firmware_build_command

        # The full network starts each peer from its factory artifact. A
        # firmware-only build can pass locally with stale artifacts but fails
        # in a fresh peer checkout before any discovery assertion executes.
        for repository in ["RFBoard26", "PowerBoard26", "FlightComputer26",
                           "gateway-board26", "ActuatorBoard26", "ValveBoard26",
                           "DAQ-Board"]:
            for release in [False, True]:
                with self.subTest(repository=repository, release=release):
                    command = network_firmware_build_command(Path("/peer"), repository, release)
                    if repository == "FlightComputer26":
                        self.assertEqual(command[-1], "factory")
                    else:
                        self.assertEqual(command[command.index("--image") + 1], "factory")

    def test_network_peers_use_the_selected_sdk_branch(self):
        from sim.run_full import network_firmware_build_command

        for repository in ["RFBoard26", "PowerBoard26", "FlightComputer26",
                           "gateway-board26", "ActuatorBoard26", "ValveBoard26",
                           "DAQ-Board"]:
            for selected in ["main", "dev"]:
                with self.subTest(repository=repository, selected=selected):
                    command = network_firmware_build_command(Path("/peer"), repository, True, selected)
                    self.assertEqual(command[command.index("--sedsnet-ref") + 1], selected)
                    self.assertEqual(command[command.index("--image") + 1], "factory")
        with self.assertRaises(ValueError):
            network_firmware_build_command(Path("/peer"), "ValveBoard26", True, "other")

    def test_ctest_zero_test_guard_is_enabled(self):
        source = Path(build.__file__).read_text(encoding="utf-8")
        self.assertIn("--no-tests=error", source)




class DaqObserverCadenceTests(unittest.TestCase):
    def test_current_and_legacy_observer_environment_matches_log_assertion(self):
        from sim import run_full
        with mock.patch.dict(run_full.os.environ, {}, clear=True):
            current = run_full.daq_loadcell_observer_config()
        self.assertEqual(current["environment"], {"GS_SIM_DAQ_LOADCELL_HZ": "40"})
        self.assertIn("40 Hz", current["assertion"]["name"])
        self.assertEqual(current["assertion"]["contains"],
                         "full-bay DAQ raw loadcell 40 Hz stream reached GroundStation")
        for rate in (1, 250, 1000):
            with self.subTest(rate=rate), mock.patch.dict(
                run_full.os.environ, {"SEDS_FIRMWARE_SIM_DAQ_LOADCELL_HZ": str(rate)}, clear=True
            ):
                legacy = run_full.daq_loadcell_observer_config()
                self.assertEqual(legacy["environment"], {"GS_SIM_DAQ_LOADCELL_HZ": str(rate)})
                self.assertIn(f"{rate} Hz", legacy["assertion"]["name"])
                self.assertIn(f"{rate} Hz", legacy["assertion"]["contains"])
                self.assertEqual(legacy["assertion"]["node"], "groundstation")

    def test_invalid_rate_fails_before_docker_readiness_or_network_work(self):
        from sim import run_full
        for value in ("", "0", "-1", "1001", "40.0", "nan", "inf", "bad"):
            with self.subTest(value=value), mock.patch.dict(
                run_full.os.environ, {"SEDS_FIRMWARE_SIM_DAQ_LOADCELL_HZ": value}, clear=True
            ), mock.patch.object(run_full.shutil, "which") as which, mock.patch.object(
                run_full.subprocess, "run"
            ) as execute:
                with self.assertRaisesRegex(RuntimeError, "SEDS_FIRMWARE_SIM_DAQ_LOADCELL_HZ"):
                    run_full.require_docker()
                which.assert_not_called()
                execute.assert_not_called()
                with mock.patch.object(run_full, "require_docker") as ready:
                    with self.assertRaisesRegex(RuntimeError, "SEDS_FIRMWARE_SIM_DAQ_LOADCELL_HZ"):
                        run_full.run_network_simulation(mock.Mock(), Path("/board"), "stm32g4")
                    ready.assert_not_called()


if __name__ == "__main__":
    unittest.main()
