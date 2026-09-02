import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class FlightStatePersistenceContract(unittest.TestCase):
    def test_flight_state_cache_is_wired_before_network_start(self):
        source = (ROOT / "Core/Src/flight_state_cache.c").read_text()
        main = (ROOT / "Core/Src/main.c").read_text()
        telemetry = (ROOT / "Core/Src/telemetry.c").read_text()
        cmake = (ROOT / "CMakeLists.txt").read_text()

        self.assertIn("FLIGHT_STATE_PERSIST_KEY", source)
        self.assertIn("persistent_store_get", source)
        self.assertIn("persistent_store_set", source)
        self.assertIn("seds_router_enable_network_variable", source)
        self.assertIn("seds_router_on_network_variable_update", source)
        self.assertIn("seds_router_seed_managed_variable_packed", source)
        self.assertLess(
            main.index("flight_state_cache_restore();"),
            main.index("MX_ThreadX_Init();"),
        )
        self.assertIn("flight_state_cache_init(r)", telemetry)
        self.assertIn("flight_state_cache_poll(g_router.r)", telemetry)
        self.assertIn("Core/Src/flight_state_cache.c", cmake)

    def test_simulator_observes_cache_and_persistence_errors(self):
        layout = json.loads((ROOT / "sim/board.json").read_text())
        probes = {
            probe["name"]: probe
            for probe in layout["execution"]["memory_probes"]
        }
        self.assertEqual(probes["flight_state_cache"]["maximum"], 15)
        self.assertEqual(probes["flight_state_errors"]["maximum"], 0)
        for name in (
            "flight_state_restores",
            "flight_state_writes",
            "flight_state_updates",
        ):
            self.assertIn(name, probes)


if __name__ == "__main__":
    unittest.main()
