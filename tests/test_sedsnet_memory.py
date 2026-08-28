import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class SedsnetMemoryTests(unittest.TestCase):
    def test_threadx_byte_pool_stays_at_known_working_size(self):
        config = (ROOT / "AZURE_RTOS" / "App" / "app_azure_rtos_config.h").read_text(
            encoding="utf-8"
        )
        size = int(re.search(r"TX_APP_MEM_POOL_SIZE\s+(\d+)", config).group(1))
        self.assertEqual(size, 55000)

    def test_shared_pool_is_not_the_legacy_per_queue_size(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        pool = int(re.search(r'set\(RF_SEDSNET_MEMORY_POOL_SIZE "(\d+)"', cmake).group(1))
        start = int(
            re.search(r'set\(RF_SEDSNET_STARTING_ALLOCATION "(\d+)"', cmake).group(1)
        )
        recent = int(re.search(r'set\(SEDSNET_MAX_RECENT_RX_IDS "(\d+)"', cmake).group(1))

        self.assertEqual(pool, 12288)
        self.assertEqual(start, 128)
        self.assertGreater(pool, recent * 8 + 2 * start)

    def test_router_uses_explicit_runtime_memory_config(self):
        source = (ROOT / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        self.assertIn("SedsRuntimeMemoryConfig memory", source)
        self.assertIn("seds_router_new_with_memory", source)
        self.assertIn(".max_queue_budget = RF_SEDSNET_MEMORY_POOL_SIZE", source)

    def test_rf_startup_leaves_headroom_for_sedsnet(self):
        radio = (ROOT / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        depth = int(re.search(r"#define RADIO_UART_TX_QUEUE_DEPTH\s+(\d+)", radio).group(1))
        self.assertEqual(depth, 16)

    def test_memory_led_requires_a_confirmed_allocator_failure(self):
        hooks = (ROOT / "Core" / "Src" / "telemetry_hooks.c").read_text(
            encoding="utf-8"
        )
        memory_branch = hooks.split("/* Prefer explicit text match if available. */", 1)[1]
        memory_branch = memory_branch.split("telemetry_panic_led_loop_memory();", 1)[0]
        self.assertIn("g_telemetry_alloc_fail", memory_branch)
        self.assertNotIn("g_last_err_memory_hint", memory_branch)


if __name__ == "__main__":
    unittest.main()
