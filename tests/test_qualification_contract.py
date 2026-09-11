import json
import unittest
from pathlib import Path

import build


class QualificationContractTests(unittest.TestCase):
    def test_gps_status_and_position_use_managed_runtime_rate(self):
        root = Path(build.__file__).resolve().parent
        source = (root / "Core/Src/neom9n_thread.c").read_text(encoding="utf-8")
        self.assertIn("rf_telemetry_period_ms()", source)
        self.assertIn('#include "telemetry_rate.h"', source)
        self.assertIn("gps_satellite_count_or_zero", source)

    def test_telemetry_stack_covers_profiled_sedsnet_call_depth(self):
        root = Path(build.__file__).resolve().parent
        source = (root / "Core" / "Src" / "telemetry_thread.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("TELEMETRY_THREAD_STACK_SIZE (14U * 1024U)", source)

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
        self.assertEqual(runner.count('str(max(1000, layout["execution"]["virtual_time_ms"]))'), 2)
        self.assertIn('"--traffic-iterations", "1000000"', runner)
        self.assertIn('"bay"', runner)
        self.assertIn('"tx_probe": "fdcan_tx_ok"', runner)
        self.assertIn('"rx_probe": "fdcan_rx"', runner)
        self.assertIn('"host_nodes"', runner)
        self.assertIn('"groundstation"', runner)
        self.assertIn('"rocket_radio"', runner)
        self.assertIn('"fill_pico"', runner)
        self.assertIn('"GS_SIM_VALIDATE_VALVE_ROUNDTRIP": "1"', runner)
        self.assertIn('"GS_SIM_VALIDATE_SOAK_COMMANDS": "1" if ultra_soak else "0"', runner)
        self.assertIn("Valve command path remained alive during soak interval", runner)
        self.assertIn("Every ten-minute soak command returned an acknowledgement", runner)
        self.assertIn('"probe": "valve_commands_received", "minimum": 1', runner)
        self.assertIn("routed status ACK toward GroundStation", runner)
        self.assertIn('simulation_env["SEDS_FIRMWARE_SIM_TEST"] = "1"', runner)
        self.assertIn('run_live(command, "firmware simulation")', runner)
        self.assertIn('running ({int(now - started)}s elapsed)', runner)
        self.assertIn('"GroundStation discovered every board by autonomous name"', runner)
        self.assertIn('"transport_path": ["RFBoard", "PowerBoard", "FlightComputer"]', runner)
        self.assertIn("rf-flight-groundstation-power-absent", runner)
        self.assertIn("RF telemetry loop remained live after FC traffic joined", runner)
        self.assertIn("RF continued transmitting radio late in the run", runner)
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
        self.assertIn("#define RF_HEALTH_PROBE __attribute__((used, externally_visible))", telemetry)
        cmake = (root / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("-Wl,--undefined=g_telemetry_peer_mask", cmake)
        self.assertIn("-Wl,--undefined=g_sim_heartbeat_attempts", cmake)
        self.assertIn("-Wl,--undefined=g_tx_drops", cmake)
        for symbol in (
            "g_telemetry_network_ready",
            "g_telemetry_discovery_seen",
            "g_telemetry_timesync_valid",
        ):
            self.assertIn(symbol, telemetry)

        can_bus = (root / "Core" / "Src" / "can_bus.c").read_text(encoding="utf-8")
        self.assertIn("g_fdcan_rx_count++", can_bus)

    def test_both_relay_sides_exist_before_timesync_startup(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        self.assertIn('r, "can", 3U, tx_send, NULL, false,', telemetry)
        prime = telemetry.index("seds_router_poll_timesync(r, &did_queue)")
        radio = telemetry.index('r, "radio", 5U, radio_tx_send')
        self.assertLess(radio, prime)
        self.assertNotIn("seds_router_process_tx_queue(r)", telemetry[radio:prime])

    def test_initial_timesync_io_backpressure_is_nonfatal(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        prime = telemetry.index("/* Prime the first source announcement")
        end = telemetry.index("g_router_retry_after_ms = 0ULL", prime)
        startup_prime = telemetry[prime:end]

        self.assertIn("seds_router_poll_timesync(r, &did_queue)", startup_prime)
        self.assertIn(
            "if (result != SEDS_OK && result != SEDS_IO)", startup_prime
        )
        self.assertIn(
            "if (result == SEDS_OK && did_queue) g_telemetry_timesync_queued++",
            startup_prime,
        )

    def test_router_clock_epoch_is_fixed_before_router_and_side_setup(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        init = telemetry.index("SedsResult init_telemetry_router(void)")
        body = telemetry[init:]
        epoch = body.index("g_router.start_time = init_now_ms;")
        create = body.index("seds_router_new_with_memory")
        can_side = body.index('r, "can", 3U, tx_send')
        radio_side = body.index('r, "radio", 5U, radio_tx_send')

        self.assertLess(epoch, create)
        self.assertLess(epoch, can_side)
        self.assertLess(epoch, radio_side)
        self.assertEqual(body.count("g_router.start_time ="), 1)

    def test_layout_uses_scheduler_as_factory_boot_success_symbol(self):
        root = Path(build.__file__).resolve().parent
        layout = json.loads((root / "sim" / "board.json").read_text(encoding="utf-8"))
        self.assertEqual(
            layout["execution"]["factory_boot_success_symbol"],
            "_tx_thread_schedule",
        )

    def test_underglow_uses_only_native_network_variable_apis(self):
        root = Path(build.__file__).resolve().parent
        source = (root / "Core" / "Src" / "av_bay_underglow.c").read_text(encoding="utf-8")
        self.assertIn("seds_router_enable_network_variable", source)
        self.assertIn("seds_router_request_managed_variable", source)
        self.assertIn("if (g_network_value_seen) return SEDS_OK;", source)
        self.assertIn("if (g_telemetry_discovery_seen == 0U) return SEDS_OK;", source)
        self.assertNotIn("seds_router_get_network_variable_packed_len", source)

        flight_state = (root / "Core" / "Src" / "flight_state_cache.c").read_text(encoding="utf-8")
        self.assertIn("seds_router_request_managed_variable", flight_state)
        self.assertIn("if (g_network_value_seen) return SEDS_OK;", flight_state)
        self.assertIn("if (g_telemetry_discovery_seen == 0U) return SEDS_OK;", flight_state)
        self.assertNotIn("seds_router_get_network_variable_packed_len", flight_state)

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
        self.assertIn("SEDS_SIDE_TRANSPORT_PROFILE_IPV6_LIKE", telemetry)
        self.assertIn("RF_SIDE_TRANSPORT_TEMPLATES 4U", telemetry)
        can_bus = (root / "Core" / "Src" / "can_bus.c").read_text(encoding="utf-8")
        self.assertIn("CAN_BUS_TX_ENQUEUE_TIMEOUT_MS 5U", can_bus)
        self.assertIn("HAL_FDCAN_AbortTxRequest", can_bus)
        self.assertNotIn("< (uint32_t)frag_cnt", can_bus)
        enqueue = can_bus.split("static HAL_StatusTypeDef can_bus_enqueue_tx_frame", 1)[1]
        enqueue = enqueue.split("static inline void can_bus_notify_rx", 1)[0]
        self.assertNotIn("for (;;)", enqueue)
        self.assertIn("HAL_GetTick()", enqueue)
        self.assertNotIn(
            "seds_router_set_route(r, g_can_side_id, g_radio_side_id, true)",
            telemetry,
        )
        self.assertNotIn("Seds_RSM_Fanout", telemetry)
        self.assertNotIn("g_bootstrap_fanout_active", telemetry)
        self.assertNotIn("seds_router_set_typed_route", telemetry)
        self.assertIn("#define RADIO_UART_MAX_PAYLOAD_SIZE    1024U", radio)

    def test_radio_hop_uses_rfd900x_link_reliability_without_nested_acks(self):
        root = Path(__file__).resolve().parents[1]
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        radio_setup = telemetry.split(
            "g_radio_side_id = seds_router_add_side_packed_profile_with_priority(", 1
        )[1].split(");", 1)[0]
        self.assertIn('r, "radio", 5U, radio_tx_send, NULL, false,', radio_setup)


    def test_periodic_health_check_does_not_serialize_topology(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        self.assertNotIn("seds_router_export_topology_len", telemetry)
        self.assertGreaterEqual(telemetry.count("g_telemetry_discovery_seen = 1U"), 2)

if __name__ == "__main__":
    unittest.main()
