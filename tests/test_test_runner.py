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


if __name__ == "__main__":
    unittest.main()
