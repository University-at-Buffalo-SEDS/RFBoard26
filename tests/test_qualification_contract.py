import json
import unittest
from pathlib import Path

import build


class QualificationContractTests(unittest.TestCase):
    def test_full_runner_profiles_memory_and_linked_network(self):
        root = Path(build.__file__).resolve().parent
        runner = (root / "sim" / "run_full.py").read_text(encoding="utf-8")
        script = (root / "build.py").read_text(encoding="utf-8")

        self.assertIn('"profile"', runner)
        self.assertIn('"--sample-count", "20"', runner)
        self.assertIn('"--traffic-iterations", "1000000"', runner)
        self.assertIn('"bay"', runner)
        self.assertIn('"activity_probe": "network_ready"', runner)
        self.assertIn('simulation_env["SEDS_FIRMWARE_SIM_TEST"] = "1"', runner)
        self.assertIn('run_live(command, "firmware simulation")', runner)
        self.assertIn('running ({int(now - started)}s elapsed)', runner)
        self.assertIn("current_build", runner)
        self.assertIn("Long-duration memory profile", script)
        self.assertIn("Network discovery and time sync", script)

    def test_layout_exposes_network_convergence(self):
        root = Path(build.__file__).resolve().parent
        layout = json.loads((root / "sim" / "board.json").read_text(encoding="utf-8"))
        self.assertLess(layout["execution"].get("memory_probe_warmup_samples", 0), layout["execution"]["sample_count"])
        probes = {
            probe["name"]: probe["symbol"]
            for probe in layout["execution"]["memory_probes"]
        }
        self.assertEqual(probes["network_ready"], "g_telemetry_network_ready")
        self.assertEqual(probes["discovery_seen"], "g_telemetry_discovery_seen")
        self.assertEqual(probes["timesync_valid"], "g_telemetry_timesync_valid")

        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        for symbol in (
            "g_telemetry_network_ready",
            "g_telemetry_discovery_seen",
            "g_telemetry_timesync_valid",
        ):
            self.assertIn(symbol, telemetry)

    def test_source_primes_can_timesync_before_radio(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        self.assertIn('seds_router_add_side_packed(r, "can", 3U, tx_send, NULL, false)', telemetry)
        prime = telemetry.index("seds_router_process_tx_queue(r)")
        radio = telemetry.index('seds_router_add_side_packed(r, "radio"')
        self.assertLess(prime, radio)

    def test_radio_side_accepts_complete_v4_topology_packets(self):
        root = Path(__file__).resolve().parents[1]
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        radio = (root / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        self.assertIn(
            'seds_router_add_side_packed(r, "radio", 5U, radio_tx_send, NULL, false)',
            telemetry,
        )
        self.assertIn("#define RADIO_UART_MAX_PAYLOAD_SIZE    1024U", radio)


    def test_periodic_health_check_does_not_serialize_topology(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        self.assertNotIn("seds_router_export_topology_len", telemetry)
        self.assertGreaterEqual(telemetry.count("g_telemetry_discovery_seen = 1U"), 2)

if __name__ == "__main__":
    unittest.main()
