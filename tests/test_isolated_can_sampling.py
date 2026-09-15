import json
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from sim import run_full


class IsolatedCanSamplingTests(unittest.TestCase):
    def test_network_sampling_preserves_warmup_and_progress_window(self):
        layouts = {"rf": {"execution": {"memory_probe_warmup_samples": 5}},
                   "daq": {"execution": {"memory_probe_warmup_samples": 4}}}
        self.assertEqual(run_full.network_sample_count(layouts, 6), 7)
        self.assertEqual(run_full.network_sample_count(layouts, 12), 12)
        self.assertEqual(layouts["rf"]["execution"]["memory_probe_warmup_samples"], 5)

    def test_progress_has_two_samples_after_warmup(self):
        for warmup in (0, 3, 4, 19):
            with self.subTest(warmup=warmup):
                layout = {"execution": {
                    "virtual_time_ms": 2000,
                    "memory_probe_warmup_samples": warmup,
                    "memory_probes": [{"name": "sample_progress",
                                      "symbol": "samples", "minimum_gain": 1}],
                }}
                captured = {}

                def capture(command, label):
                    mount = next(value for value in command
                                 if value.endswith(":/simulation:ro"))
                    captured["layout"] = json.loads(
                        (Path(mount.removesuffix(":/simulation:ro")) /
                         "board.json").read_text())
                    captured["count"] = int(command[command.index("--sample-count") + 1])

                with patch.object(run_full, "require_docker", return_value="docker"), \
                     patch.object(run_full, "resolve_simulator_image", return_value="sim:test"), \
                     patch.object(run_full, "load_layout_for_build", return_value=layout), \
                     patch.object(run_full, "run_live", side_effect=capture):
                    run_full.run_unacknowledged_can_simulation(
                        Mock(), Path(__file__).resolve().parents[1], "stm32g4")

                execution = captured["layout"]["execution"]
                self.assertGreaterEqual(
                    captured["count"] - execution["memory_probe_warmup_samples"], 2)
                self.assertEqual(execution["memory_probes"][0]["minimum_gain"], 1)


if __name__ == "__main__":
    unittest.main()
