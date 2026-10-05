"""Exercise topology construction, rather than only searching source strings."""
import copy
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from sim import run_full

REPOSITORIES = ("RFBoard26", "PowerBoard26", "FlightComputer26", "gateway-board26",
                "ActuatorBoard26", "ValveBoard26", "DAQ-Board")
ROOT = Path(__file__).resolve().parents[1]

class NetworkRestartScenarioTests(unittest.TestCase):
    def capture(self, ultra_soak):
        captured = []
        layout = {"execution": {"memory_probes": [], "sample_count": 6,
                                "memory_probe_warmup_samples": 0, "virtual_time_ms": 16000}}
        with tempfile.TemporaryDirectory() as tmp:
            for repository in REPOSITORIES:
                (Path(tmp) / repository / ".git").mkdir(parents=True)
            def capture(command, label):
                mount = next(item for item in command if item.endswith(":/simulation:ro"))
                captured.append(json.loads((Path(mount.rsplit(":", 2)[0]) / "topology.json").read_text()))
            with mock.patch.dict(os.environ, {"SEDS_FIRMWARE_SIM_SUITE_ROOT": tmp,
                                               "SEDS_FIRMWARE_SIM_SKIP_BUILD": "1"}, clear=True), \
                 mock.patch.object(run_full, "require_docker", return_value="docker"), \
                 mock.patch.object(run_full, "resolve_simulator_image", return_value="sim:test"), \
                 mock.patch.object(run_full.shutil, "which", return_value="git"), \
                 mock.patch.object(run_full, "load_layout_for_build", side_effect=lambda *a: copy.deepcopy(layout)), \
                 mock.patch.object(run_full, "run_live", side_effect=capture):
                run_full.run_network_simulation(SimpleNamespace(say=lambda *a: None),
                                                ROOT, "stm32g4", "Release", ultra_soak=ultra_soak)
        return captured

    def test_short_gate_has_no_reboots(self):
        topologies = self.capture(False)
        self.assertTrue(topologies)
        self.assertTrue(all(not t.get("reboots") for t in topologies))

    def test_long_gate_restarts_service_without_power_cycling_boards_first(self):
        topology = self.capture(True)[-1]
        self.assertEqual(topology["virtual_time_ms"], 600000)
        first = topology["sample_count"] // 3
        later = topology["sample_count"] * 2 // 3
        self.assertEqual([e["node"] for e in topology["reboots"] if e["after_sample"] == first],
                         ["groundstation"])
        self.assertEqual({e["node"] for e in topology["reboots"] if e["after_sample"] == later},
                         {"groundstation", "rf", "power", "flight"})
        for assertion in topology["host_log_assertions"]:
            if assertion["name"] in ("GroundStation discovered every board by autonomous name",
                                     "GroundStation attributed traffic to every board identity",
                                     "GroundStation network graph labelled every board with its own traffic"):
                self.assertEqual(assertion["minimum_occurrences"], 3)
        self.assertTrue(any("Every ten-minute soak command" in a["name"]
                            for a in topology["host_log_assertions"]))

