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
        budget = int(re.search(r'set\(RF_SEDSNET_QUEUE_BUDGET "(\d+)"', cmake).group(1))
        start = int(
            re.search(r'set\(RF_SEDSNET_STARTING_ALLOCATION "(\d+)"', cmake).group(1)
        )
        recent = int(re.search(r'set\(SEDSNET_MAX_RECENT_RX_IDS "(\d+)"', cmake).group(1))

        self.assertEqual(pool, 28672)
        self.assertEqual(budget, 12288)
        self.assertGreaterEqual(pool - budget, 16384)
        self.assertEqual(start, 1024)
        self.assertGreater(budget, recent * 8 + start)

    def test_reclaimed_stack_space_is_guarded_by_threadx(self):
        telemetry_thread = (ROOT / "Core" / "Src" / "telemetry_thread.c").read_text(
            encoding="utf-8"
        )
        threadx = (ROOT / "Core" / "Inc" / "tx_user.h").read_text(encoding="utf-8")
        self.assertIn("TELEMETRY_THREAD_STACK_SIZE (16U * 1024U)", telemetry_thread)
        gps_thread = (ROOT / "Core" / "Src" / "neom9n_thread.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("NEOM9N_THREAD_STACK_SIZE (9U * 1024U)", gps_thread)
        self.assertRegex(threadx, r"(?m)^#define TX_ENABLE_STACK_CHECKING$")
        app = (ROOT / "Core" / "Src" / "app_threadx.c").read_text(encoding="utf-8")
        self.assertIn("g_thread_stack_error_count", app)
        self.assertIn("tx_thread_stack_error_notify(thread_stack_error_handler)", app)

    def test_normal_telemetry_does_not_use_the_fallback_tx_queue(self):
        source = (ROOT / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        async_body = source.split("SedsResult log_telemetry_asynchronous", 1)[1]
        async_body = async_body.split("SedsResult dispatch_tx_queue", 1)[0]
        self.assertIn("seds_router_log_typed", async_body)
        self.assertNotIn("seds_router_log_queue_typed", async_body)
        self.assertIn("strlen(str), NULL, 0", async_body)

    def test_router_uses_explicit_runtime_memory_config(self):
        source = (ROOT / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        self.assertIn("SedsRuntimeMemoryConfig memory", source)
        self.assertIn("seds_router_new_with_memory", source)
        self.assertIn(".max_queue_budget = RF_SEDSNET_QUEUE_BUDGET", source)
        self.assertIn(".queue_grow_step = 1.0", source)

    def test_sedsnet_uses_an_isolated_threadx_pool(self):
        source = (ROOT / "Core" / "Src" / "app_threadx.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("static TX_BYTE_POOL sedsnet_byte_pool", source)
        self.assertIn("RF_SEDSNET_MEMORY_POOL_SIZE, TX_NO_WAIT", source)
        self.assertIn("tx_byte_pool_create(&sedsnet_byte_pool", source)
        self.assertIn("telemetry_set_byte_pool(&sedsnet_byte_pool)", source)

    def test_rf_startup_leaves_headroom_for_sedsnet(self):
        radio = (ROOT / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        depth = int(re.search(r"#define RADIO_UART_TX_QUEUE_DEPTH\s+(\d+)", radio).group(1))
        self.assertEqual(depth, 16)
        self.assertIn(
            "g_tx_queue_storage[RADIO_UART_TX_QUEUE_DEPTH]", radio
        )
        init = radio.split("UINT radio_uart_init_tx_queue", 1)[1]
        init = init.split("/* Arm RX-to-idle", 1)[0]
        self.assertNotIn("tx_byte_allocate", init)

    def test_memory_led_requires_a_confirmed_allocator_failure(self):
        hooks = (ROOT / "Core" / "Src" / "telemetry_hooks.c").read_text(
            encoding="utf-8"
        )
        memory_branch = hooks.split("/* Prefer explicit text match if available. */", 1)[1]
        memory_branch = memory_branch.split("telemetry_panic_led_loop_memory();", 1)[0]
        self.assertIn("g_telemetry_alloc_fail", memory_branch)
        self.assertNotIn("g_last_err_memory_hint", memory_branch)

    def test_allocator_profile_records_low_water_and_init_stages(self):
        hooks = (ROOT / "Core" / "Src" / "telemetry_hooks.c").read_text(
            encoding="utf-8"
        )
        telemetry = (ROOT / "Core" / "Src" / "telemetry.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("g_telemetry_pool_low_water", hooks)
        self.assertIn("g_telemetry_max_alloc_request", hooks)
        self.assertIn("g_telemetry_profile_available[8]", hooks)
        for stage in range(7):
            self.assertIn(f"telemetry_memory_profile_mark({stage}U)", telemetry)

    def test_failed_router_initialization_is_rate_limited(self):
        source = (ROOT / "Core" / "Src" / "telemetry.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("TELEMETRY_ROUTER_RETRY_MS", source)
        self.assertIn("init_now_ms < g_router_retry_after_ms", source)
        self.assertGreaterEqual(
            source.count("g_router_retry_after_ms = init_now_ms +"), 3
        )

    def test_receive_callbacks_dispatch_without_a_scheduler_queue(self):
        source = (ROOT / "Core" / "Src" / "telemetry.c").read_text(
            encoding="utf-8"
        )
        callbacks = source.split("static void telemetry_can_rx", 1)[1]
        callbacks = callbacks.split("SedsResult telemetry_poll_timesync", 1)[0]
        self.assertIn("seds_router_receive_packed_from_side", callbacks)
        self.assertNotIn("rx_packed_packet_to_queue", callbacks)

        thread = (ROOT / "Core" / "Src" / "telemetry_thread.c").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("process_rx_queue_timeout", thread)


if __name__ == "__main__":
    unittest.main()
