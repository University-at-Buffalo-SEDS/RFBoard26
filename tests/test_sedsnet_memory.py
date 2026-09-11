import re
import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class SedsnetMemoryTests(unittest.TestCase):
    def test_simulator_qualifies_the_shared_allocator_not_individual_arenas(self):
        layout = json.loads((ROOT / "sim" / "board.json").read_text(encoding="utf-8"))
        probes = {probe["name"]: probe for probe in layout["execution"]["memory_probes"]}

        self.assertEqual(probes["pool_available"]["minimum"], 2560)
        self.assertEqual(probes["pool_available"]["max_end_drop"], 8192)
        self.assertEqual(probes["pool_low_water"]["minimum"], 1024)
        self.assertEqual(probes["alloc_failures"]["maximum"], 0)
        self.assertEqual(probes["panics"]["maximum"], 0)
        self.assertNotIn("minimum", probes["small_pool_available"])
        self.assertNotIn("minimum", probes["large_pool_available"])

    def test_radio_rx_service_is_bounded_before_router_and_tx(self):
        radio = (ROOT / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        thread = (ROOT / "Core" / "Src" / "telemetry_thread.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("RADIO_UART_RX_SERVICE_BUDGET", radio)
        self.assertIn("processed < RADIO_UART_RX_SERVICE_BUDGET", radio)
        self.assertIn("g_radio_rx_service_budget_hits", radio)
        self.assertLess(
            thread.index("radio_uart_process_rx();"),
            thread.index("g_telemetry_loop_completions++"),
        )
        self.assertLess(
            thread.index("g_telemetry_loop_completions++"),
            thread.index("tx_thread_sleep(TELEMETRY_THREAD_SLEEP_TICKS)"),
        )

    def test_can_tx_event_drain_cannot_starve_radio_egress(self):
        can_bus = (ROOT / "Core" / "Src" / "can_bus.c").read_text(encoding="utf-8")
        self.assertIn("CAN_BUS_TX_EVENT_SERVICE_BUDGET", can_bus)
        self.assertIn(
            "tx_events_processed < CAN_BUS_TX_EVENT_SERVICE_BUDGET",
            can_bus,
        )

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

        self.assertEqual(pool, 67072)
        self.assertEqual(budget, 6144)
        self.assertGreaterEqual(pool - budget, 16384)
        self.assertEqual(start, 512)
        self.assertGreater(budget, recent * 8 + start)

    def test_release_disables_usb_and_gives_its_headroom_to_router(self):
        config = (ROOT / "AZURE_RTOS" / "App" / "app_azure_rtos_config.h").read_text(
            encoding="utf-8"
        )
        usbx = (ROOT / "USBX" / "App" / "app_usbx_device.h").read_text(
            encoding="utf-8"
        )
        ioc = (ROOT / "RFBoard26.ioc").read_text(encoding="utf-8")
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

        ux_user = (ROOT / "USBX" / "App" / "ux_user.h").read_text(
            encoding="utf-8"
        )
        azure = (ROOT / "AZURE_RTOS" / "App" / "app_azure_rtos.c").read_text(
            encoding="utf-8"
        )
        main = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")

        self.assertIn("RF_USB_DEBUG_ENABLED=$<IF:$<CONFIG:Debug>,1,0>", cmake)
        self.assertIn("#if RF_USB_DEBUG_ENABLED", azure)
        self.assertIn("#if RF_USB_DEBUG_ENABLED", main)

        irq = (ROOT / "Core" / "Src" / "stm32g4xx_it.c").read_text(
            encoding="utf-8"
        )
        usb_handler = irq.split("void USB_LP_IRQHandler(void)", 1)[1]
        usb_handler = usb_handler.split("void USART1_IRQHandler(void)", 1)[0]
        self.assertIn("#if RF_USB_DEBUG_ENABLED", usb_handler)
        self.assertIn("HAL_PCD_IRQHandler(&hpcd_USB_FS);", usb_handler)
        self.assertIn("UX_DEVICE_APP_MEM_POOL_SIZE              10752", config)
        self.assertIn("USBX_DEVICE_MEMORY_STACK_SIZE       8192", usbx)
        self.assertIn("UX_DEVICE_APP_THREAD_STACK_SIZE   2048", usbx)
        self.assertIn("UX_THREAD_STACK_SIZE                                2048", ux_user)
        self.assertIn("USBX_DEVICE_APP_THREAD_Size=2048", ioc)
        self.assertIn("USBX_DEVICE_SYS_SIZE=8192", ioc)
        self.assertIn("UX_DEVICE_APP_MEM_POOL_SIZE=10752", ioc)
        self.assertIn("UX_THREAD_STACK_SIZE=2048", ioc)
        self.assertIn('RF_SEDSNET_EMERGENCY_POOL_SIZE "19456"', cmake)

    def test_reclaimed_stack_space_is_guarded_by_threadx(self):
        telemetry_thread = (ROOT / "Core" / "Src" / "telemetry_thread.c").read_text(
            encoding="utf-8"
        )
        threadx = (ROOT / "Core" / "Inc" / "tx_user.h").read_text(encoding="utf-8")
        self.assertIn("TELEMETRY_THREAD_STACK_SIZE (14U * 1024U)", telemetry_thread)
        gps_thread = (ROOT / "Core" / "Src" / "neom9n_thread.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("NEOM9N_THREAD_STACK_SIZE (12U * 1024U)", gps_thread)
        self.assertIn("static ULONG neom9n_thread_stack", gps_thread)
        self.assertIn("(void)byte_pool;", gps_thread)
        self.assertIn(
            "g_neom9n_stack_remaining = NEOM9N_THREAD_STACK_SIZE;", gps_thread
        )
        self.assertGreaterEqual(gps_thread.count("neom9n_update_stack_profile();"), 2)
        self.assertIn("g_neom9n_stack_current = __get_PSP();", gps_thread)
        self.assertIn("gps_satellite_count_or_zero", gps_thread)
        self.assertIn("return 0U;", gps_thread)
        self.assertIn("rf_telemetry_period_ms()", gps_thread)
        self.assertRegex(threadx, r"(?m)^#define TX_ENABLE_STACK_CHECKING$")
        app = (ROOT / "Core" / "Src" / "app_threadx.c").read_text(encoding="utf-8")
        self.assertIn("g_thread_stack_error_count", app)
        self.assertIn("g_thread_stack_error_thread", app)
        self.assertIn("thread_ptr->tx_thread_stack_start", app)
        self.assertIn("thread_ptr->tx_thread_stack_end", app)
        self.assertIn("tx_thread_stack_error_notify(thread_stack_error_handler)", app)

    def test_hardfault_capture_uses_the_exception_frame_stack(self):
        handler = (ROOT / "Core" / "Src" / "stm32g4xx_it.c").read_text()
        self.assertIn('"tst r1, #4', handler)
        self.assertIn('"mrseq r0, msp', handler)
        self.assertIn('"mrsne r0, psp', handler)
        self.assertIn("g_hardfault_fault_stack", handler)
        self.assertIn("g_hardfault_core_frame", handler)
        self.assertIn("core_frame += 18U", handler)
        self.assertIn("__attribute__((naked)) void HardFault_Handler", handler)
        self.assertIn("hardfault_capture_and_halt", handler)

    def test_legacy_radio_scheduler_is_removed_and_stack_health_is_early(self):
        thread = (ROOT / "Core" / "Src" / "telemetry_thread.c").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("RADIO_SCHED", thread)
        entry = thread.split("void telemetry_thread_entry", 1)[1]
        self.assertLess(
            entry.index("telemetry_update_stack_profile();"),
            entry.index("init_telemetry_router();"),
        )

    def test_radio_ingress_is_only_handed_to_sedsnet(self):
        source = (ROOT / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        receive = source.split("static void telemetry_radio_rx", 1)[1]
        receive = receive.split("void rx_asynchronous", 1)[0]
        self.assertIn("seds_router_receive_packed_from_side", receive)
        self.assertNotIn("can_bus_send_large", receive)
        self.assertNotIn("telemetry_send_or_queue_can", receive)

    def test_gps_uses_sedsnet_routing_instead_of_driver_fanout(self):
        source = (ROOT / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        async_body = source.split("SedsResult log_telemetry_asynchronous", 1)[1]
        async_body = async_body.split("SedsResult log_telemetry_string_asynchronous", 1)[0]
        self.assertIn("seds_router_log_typed", async_body)
        self.assertNotIn("radio_uart_send_bytes", async_body)
        self.assertNotIn("can_bus_send_large", async_body)

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
        self.assertIn("static TX_BYTE_POOL sedsnet_large_byte_pool", source)
        self.assertIn("static TX_BYTE_POOL sedsnet_emergency_byte_pool", source)
        self.assertIn("primary_pool_size, TX_NO_WAIT", source)
        self.assertIn("tx_byte_pool_create(&sedsnet_byte_pool", source)
        self.assertIn("tx_byte_pool_create(&sedsnet_large_byte_pool", source)
        self.assertIn("telemetry_set_byte_pool(&sedsnet_byte_pool)", source)
        self.assertIn("telemetry_set_large_byte_pool(&sedsnet_large_byte_pool)", source)
        self.assertIn("telemetry_set_emergency_byte_pool(&sedsnet_emergency_byte_pool)", source)

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
        self.assertIn('"symbol": "g_tx_drops"', layout)

    def test_sedsnet_radio_frames_preserve_scheduler_order(self):
        radio = (ROOT / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        telemetry = (ROOT / "Core" / "Src" / "telemetry.c").read_text(
            encoding="utf-8"
        )
        send = radio.split("HAL_StatusTypeDef radio_uart_send_bytes_priority", 1)[1]
        send = send.split("HAL_StatusTypeDef radio_uart_send_plaintext", 1)[0]
        self.assertIn("radio_uart_enqueue_sedsnet_frame", send)
        self.assertNotIn("HAL_UART_Transmit(", send)
        self.assertNotIn("radio_uart_enqueue_frame", send)
        self.assertNotIn("radio_uart_air_busy", send)
        self.assertNotIn("radio_uart_mark_tx_quiet", send)
        self.assertIn("seds_router_add_side_packed_profile_with_priority", telemetry)
        self.assertIn("radio_uart_send_bytes_priority(bytes, len, priority)", telemetry)

    def test_rfd900_dma_is_not_delayed_by_the_legacy_lora_airtime_scheduler(self):
        radio = (ROOT / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        process = radio.split("uint32_t radio_uart_process_tx_with_budget", 1)[1]
        process = process.split("HAL_StatusTypeDef radio_uart_subscribe_rx", 1)[0]
        self.assertIn("HAL_UART_Transmit_DMA", process)
        self.assertNotIn("radio_uart_reserve_airtime", process)
        self.assertNotIn("radio_uart_available_air_budget", process)

    def test_rfd900_dma_failure_cannot_poison_or_permanently_stall_compact_tx(self):
        radio = (ROOT / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        process = radio.split("uint32_t radio_uart_process_tx_with_budget", 1)[1]
        process = process.split("HAL_StatusTypeDef radio_uart_subscribe_rx", 1)[0]
        self.assertIn("radio_uart_requeue_frame_front(&g_tx_dma_item)", process)
        self.assertIn("elapsed >= deadline", process)
        self.assertIn("HAL_UART_AbortTransmit(g_huart)", process)
        error_callback = radio.split("void HAL_UART_ErrorCallback", 1)[1]
        self.assertNotIn("HAL_UART_AbortTransmit(huart)", error_callback)

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

    def test_radio_preserves_the_order_selected_by_sedsnet(self):
        radio = (ROOT / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        dequeue = radio.split("static uint8_t radio_uart_dequeue_frame_with_budget", 1)[1]
        dequeue = dequeue.split("static uint32_t radio_uart_tx_timeout_ms", 1)[0]
        self.assertIn("const uint32_t selected = 0U", dequeue)
        self.assertNotIn("selected_priority", dequeue)

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

    def test_allocator_segregates_large_buffers_from_small_routing_records(self):
        hooks = (ROOT / "Core" / "Src" / "telemetry_hooks.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("TELEMETRY_LARGE_ALLOCATION_THRESHOLD 1024U", hooks)
        self.assertIn("rust_large_byte_pool_external", hooks)
        self.assertIn("selected_pool = rust_large_byte_pool_external", hooks)
        self.assertIn("alternate_pool", hooks)
        self.assertIn("g_telemetry_alloc_cross_pool_recoveries++", hooks)
        self.assertIn("g_telemetry_large_pool_available", hooks)
        self.assertIn("xSize, TX_NO_WAIT", hooks)
        self.assertNotRegex(hooks, r"tx_byte_allocate\([^;]+,\s*5\s*\)")
        self.assertIn("g_telemetry_alloc_failure_system_state", hooks)
        self.assertIn("g_telemetry_alloc_failure_status", hooks)

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
