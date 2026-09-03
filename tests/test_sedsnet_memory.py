import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class SedsnetMemoryTests(unittest.TestCase):
    def test_pending_can_retry_dequeues_each_packet_exactly_once(self):
        source = (ROOT / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        retry = source.split("void telemetry_retry_pending_can_commands(void)", 1)[1]
        retry = retry.split("static bool telemetry_unix_ms_to_utc", 1)[0]
        self.assertEqual(retry.count("g_pending_can_count--;"), 1)

    def test_threadx_byte_pool_stays_at_known_working_size(self):
        config = (ROOT / "AZURE_RTOS" / "App" / "app_azure_rtos_config.h").read_text(
            encoding="utf-8"
        )
        size = int(re.search(r"TX_APP_MEM_POOL_SIZE\s+(\d+)", config).group(1))
        self.assertEqual(size, 66264)
        ioc = (ROOT / "RFBoard26.ioc").read_text(encoding="utf-8")
        self.assertIn("TX_APP_MEM_POOL_SIZE=66264", ioc)

    def test_shared_pool_is_not_the_legacy_per_queue_size(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        pool = int(re.search(r'set\(RF_SEDSNET_MEMORY_POOL_SIZE "(\d+)"', cmake).group(1))
        budget = int(re.search(r'set\(RF_SEDSNET_QUEUE_BUDGET "(\d+)"', cmake).group(1))
        start = int(
            re.search(r'set\(RF_SEDSNET_STARTING_ALLOCATION "(\d+)"', cmake).group(1)
        )
        recent = int(re.search(r'set\(SEDSNET_MAX_RECENT_RX_IDS "(\d+)"', cmake).group(1))

        self.assertEqual(pool, 48096)
        self.assertEqual(budget, 8192)
        self.assertGreaterEqual(pool - budget, 16384)
        self.assertEqual(start, 512)
        self.assertGreater(budget, recent * 8 + start)

    def test_reclaimed_stack_space_is_guarded_by_threadx(self):
        telemetry_thread = (ROOT / "Core" / "Src" / "telemetry_thread.c").read_text(
            encoding="utf-8"
        )
        threadx = (ROOT / "Core" / "Inc" / "tx_user.h").read_text(encoding="utf-8")
        self.assertIn("TELEMETRY_THREAD_STACK_SIZE (12U * 1024U)", telemetry_thread)
        gps_thread = (ROOT / "Core" / "Src" / "neom9n_thread.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("NEOM9N_THREAD_STACK_SIZE (5U * 1024U)", gps_thread)
        self.assertRegex(threadx, r"(?m)^#define TX_ENABLE_STACK_CHECKING$")
        app = (ROOT / "Core" / "Src" / "app_threadx.c").read_text(encoding="utf-8")
        self.assertIn("g_thread_stack_error_count", app)
        self.assertIn("tx_thread_stack_error_notify(thread_stack_error_handler)", app)

    def test_half_duplex_radio_scheduler_is_enabled_and_stack_health_is_early(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        thread = (ROOT / "Core" / "Src" / "telemetry_thread.c").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            "set(ENABLE_RADIO_SCHEDULER ON CACHE BOOL",
            cmake,
        )
        entry = thread.split("void telemetry_thread_entry", 1)[1]
        self.assertLess(
            entry.index("telemetry_update_stack_profile();"),
            entry.index("init_telemetry_router();"),
        )

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
        payload = int(
            re.search(r"#define RADIO_UART_MAX_PAYLOAD_SIZE\s+(\d+)U", radio).group(1)
        )
        self.assertEqual(depth, 2)
        self.assertGreaterEqual(payload, 512)
        self.assertLessEqual(depth * (payload + 4), 4200)
        self.assertIn(
            "static radio_tx_queue_item_t g_tx_queue[RADIO_UART_TX_QUEUE_SLOTS]",
            radio,
        )
        self.assertIn(
            "static uint8_t g_tx_queue_bytes[RADIO_UART_TX_QUEUE_BYTE_CAPACITY]",
            radio,
        )
        telemetry = (ROOT / "Core" / "Src" / "telemetry.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("#define RF_SEDSNET_QUEUE_BUDGET 9984U", telemetry)
        self.assertIn("#define TELEMETRY_PENDING_CAN_DEPTH 3U", telemetry)
        self.assertIn("#define TELEMETRY_PENDING_CAN_MAX_LEN 128U", telemetry)
        init = radio.split("UINT radio_uart_init_tx_queue", 1)[1]
        init = init.split("/* Arm RX-to-idle", 1)[0]
        self.assertNotIn("tx_byte_allocate", init)
        self.assertIn("memset(g_tx_queue, 0, sizeof(g_tx_queue))", init)

    def test_radio_coalescing_is_not_reported_as_application_loss(self):
        radio = (ROOT / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        coalescing_branch = radio.split(
            "if ((incoming_control_class != RADIO_UART_CLASS_APPLICATION", 1
        )[1].split("while (g_tx_count", 1)[0]
        self.assertIn("g_tx_drop_same_flow++", coalescing_branch)
        self.assertNotIn("g_tx_drop_application++", coalescing_branch)
        self.assertIn("incoming_is_heartbeat", coalescing_branch)

    def test_expected_heartbeat_eviction_is_separate_from_application_loss(self):
        radio = (ROOT / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        self.assertIn("volatile uint32_t g_tx_drop_heartbeat = 0U", radio)
        classifier = radio.split("static void radio_uart_classify_packet_drop", 1)[1]
        classifier = classifier.split("static void", 1)[0]
        self.assertIn("else if (is_heartbeat)", classifier)
        self.assertIn("g_tx_drop_heartbeat++", classifier)
        self.assertIn("g_tx_drop_application++", classifier)

        layout = (ROOT / "sim" / "board.json").read_text(encoding="utf-8")
        self.assertIn('"symbol": "g_tx_drop_heartbeat"', layout)
        self.assertIn('"symbol": "g_tx_drop_application"', layout)
        self.assertIn('"symbol": "g_tx_drop_control"', layout)
        self.assertIn('"symbol": "g_tx_drop_discovery"', layout)
        self.assertIn('"symbol": "g_tx_drop_network_control"', layout)

    def test_discovery_cannot_be_starved_by_normal_radio_traffic(self):
        radio = (ROOT / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        self.assertIn("RADIO_UART_PRIORITY_DISCOVERY       255U", radio)
        self.assertIn("data_type >= 7ULL && data_type <= 12ULL", radio)
        self.assertIn("data_type >= 15ULL && data_type <= 17ULL", radio)
        self.assertIn("if (incoming_priority < lowest_priority)", radio)
        self.assertIn(
            "lowest_priority >= RADIO_UART_PRIORITY_NETWORK_CONTROL", radio
        )
        telemetry = (ROOT / "Core" / "Src" / "telemetry.c").read_text(
            encoding="utf-8"
        )
        self.assertNotIn(
            "status == HAL_BUSY && !radio_uart_tx_ready()", telemetry
        )

    def test_radio_priority_reserves_network_control_then_uses_schema_order(self):
        radio = (ROOT / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        self.assertIn("RADIO_UART_PRIORITY_NETWORK_CONTROL 254U", radio)
        self.assertIn("data_type >= 4ULL && data_type <= 6ULL", radio)
        self.assertIn("data_type == 13ULL || data_type == 14ULL", radio)
        self.assertIn("SEDS_DT_FLIGHT_STATE", radio)
        self.assertIn("SEDS_DT_AV_BAY_UNDERGLOW", radio)
        self.assertIn("seds_dtype_get_info", radio)
        self.assertIn("info.priority > RADIO_UART_PRIORITY_USER_MAX", radio)

    def test_radio_uses_sedsnet_adaptive_discovery_cadence(self):
        telemetry_thread = (
            ROOT / "Core" / "Src" / "telemetry_thread.c"
        ).read_text(encoding="utf-8")
        self.assertIn("telemetry_poll_discovery();", telemetry_thread)
        self.assertNotIn("telemetry_announce_discovery_if_due", telemetry_thread)
        self.assertNotIn("TELEMETRY_DISCOVERY_ANNOUNCE_INTERVAL_MS", telemetry_thread)

    def test_radio_dequeues_highest_priority_packet_first(self):
        radio = (ROOT / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        dequeue = radio.split("static uint8_t radio_uart_dequeue_frame_with_budget", 1)[1]
        dequeue = dequeue.split("static uint32_t radio_uart_tx_timeout_ms", 1)[0]
        self.assertIn("g_tx_queue[idx].priority > selected_priority", dequeue)

    def test_radio_does_not_hold_discovery_behind_a_startup_delay(self):
        radio = (ROOT / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        main = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")
        delay = int(
            re.search(r"#define RADIO_UART_TX_STARTUP_DELAY_MS\s+(\d+)U", radio).group(1)
        )
        self.assertLessEqual(delay, 10)
        self.assertNotIn("huart1.Init.BaudRate = 115200", main)
        self.assertIn("huart1.Init.BaudRate = RADIO_BAUD_RATE", main)

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

    def test_allocator_panic_reports_snapshot_over_usb_cdc(self):
        hooks = (ROOT / "Core" / "Src" / "telemetry_hooks.c").read_text(
            encoding="utf-8"
        )
        report = hooks.index("SEDSNet panic: request=")
        led_loop = hooks.index("telemetry_panic_led_loop_memory();", report)
        self.assertLess(report, led_loop)
        for symbol in (
            "g_telemetry_alloc_failure_request",
            "g_telemetry_alloc_failure_available",
            "g_telemetry_alloc_failure_fragments",
            "g_telemetry_pool_low_water",
        ):
            self.assertIn(symbol, hooks[report:led_loop])


if __name__ == "__main__":
    unittest.main()
