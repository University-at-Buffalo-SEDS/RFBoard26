import contextlib
import io
import unittest
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

    def test_ctest_zero_test_guard_is_enabled(self):
        source = open(build.__file__, encoding="utf-8").read()
        self.assertIn("--no-tests=error", source)


if __name__ == "__main__":
    unittest.main()

