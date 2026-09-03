import json
import unittest
from pathlib import Path

import build


class QualificationContractTests(unittest.TestCase):
    def test_telemetry_stack_covers_profiled_sedsnet_call_depth(self):
        root = Path(build.__file__).resolve().parent
        source = (root / "Core" / "Src" / "telemetry_thread.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("TELEMETRY_THREAD_STACK_SIZE (12U * 1024U)", source)

    def test_can_transport_starts_before_router(self):
        root = Path(build.__file__).resolve().parent
        source = (root / "Core" / "Src" / "telemetry_thread.c").read_text(
            encoding="utf-8"
        )
        self.assertLess(
            source.index("can_bus_init(&hfdcan2);"),
            source.index("init_telemetry_router();"),
        )

    def test_full_runner_profiles_memory_and_linked_network(self):
        root = Path(build.__file__).resolve().parent
        runner = (root / "sim" / "run_full.py").read_text(encoding="utf-8")
        script = (root / "build.py").read_text(encoding="utf-8")

        self.assertIn('"profile"', runner)
        self.assertIn('"--sample-count", "20"', runner)
        self.assertIn('"--traffic-iterations", "1000000"', runner)
        self.assertIn('"bay"', runner)
        self.assertIn('"tx_probe": "fdcan_tx_ok"', runner)
        self.assertIn('"rx_probe": "fdcan_rx"', runner)
        self.assertIn('"host_nodes"', runner)
        self.assertIn('"groundstation"', runner)
        self.assertIn('"rocket_radio"', runner)
        self.assertIn('"fill_pico"', runner)
        self.assertIn('"GS_SIM_VALIDATE_VALVE_ROUNDTRIP": "1"', runner)
        self.assertIn('"probe": "valve_commands_received", "minimum": 1', runner)
        self.assertIn("forwarded status ACK to GroundStation", runner)
        self.assertIn('simulation_env["SEDS_FIRMWARE_SIM_TEST"] = "1"', runner)
        self.assertIn('run_live(command, "firmware simulation")', runner)
        self.assertIn('running ({int(now - started)}s elapsed)', runner)
        self.assertIn('"rf decoded both avionics peers"', runner)
        self.assertIn('"rf decoded both avionics peers"', runner)
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
        self.assertEqual(probes["fdcan_rx"], "g_fdcan_rx_count")

        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        for symbol in (
            "g_telemetry_network_ready",
            "g_telemetry_discovery_seen",
            "g_telemetry_timesync_valid",
        ):
            self.assertIn(symbol, telemetry)

        can_bus = (root / "Core" / "Src" / "can_bus.c").read_text(encoding="utf-8")
        self.assertIn("g_fdcan_rx_count++", can_bus)

    def test_source_queues_can_timesync_before_radio_without_blocking_startup(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        self.assertIn('r, "can", 3U, tx_send, NULL, false,', telemetry)
        prime = telemetry.index("seds_router_poll_timesync(r, &did_queue)")
        radio = telemetry.index('r, "radio", 5U, radio_tx_send')
        self.assertLess(prime, radio)
        self.assertNotIn("seds_router_process_tx_queue(r)", telemetry[prime:radio])

    def test_underglow_uses_only_native_network_variable_apis(self):
        root = Path(build.__file__).resolve().parent
        source = (root / "Core" / "Src" / "av_bay_underglow.c").read_text(encoding="utf-8")
        self.assertIn("seds_router_enable_network_variable", source)
        self.assertIn("seds_router_get_network_variable_packed_len", source)
        self.assertNotIn("seds_router_request_managed_variable", source)

    def test_radio_side_chunks_complete_v4_topology_packets(self):
        root = Path(__file__).resolve().parents[1]
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        radio = (root / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        self.assertIn(
            "seds_router_add_side_packed_profile(",
            telemetry,
        )
        self.assertIn("RF_RADIO_MAX_FRAME_BYTES 1024U", telemetry)
        self.assertIn("RF_CAN_MAX_FRAME_BYTES 128U", telemetry)
        self.assertIn("RF_SIDE_TRANSPORT_TEMPLATES 4U", telemetry)
        self.assertNotIn(
            "seds_router_set_route(r, g_can_side_id, g_radio_side_id, true)",
            telemetry,
        )
        self.assertNotIn("Seds_RSM_Fanout", telemetry)
        self.assertNotIn("g_bootstrap_fanout_active", telemetry)
        self.assertNotIn("seds_router_set_typed_route", telemetry)
        self.assertIn("#define RADIO_UART_MAX_PAYLOAD_SIZE    1024U", radio)


    def test_periodic_health_check_does_not_serialize_topology(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        self.assertNotIn("seds_router_export_topology_len", telemetry)
        self.assertGreaterEqual(telemetry.count("g_telemetry_discovery_seen = 1U"), 2)

if __name__ == "__main__":
    unittest.main()
