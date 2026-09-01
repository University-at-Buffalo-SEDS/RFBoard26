import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class MemoryRegressionTests(unittest.TestCase):
    def test_can_does_not_retain_application_retransmit_copies(self):
        telemetry = (ROOT / "Core" / "Src" / "telemetry.c").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            'seds_router_add_side_serialized(r, "can", 3U, tx_send, NULL, false)',
            telemetry,
        )

    def test_inline_payload_setting_does_not_limit_wire_packet_size(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        radio = (ROOT / "Core" / "Src" / "radio.c").read_text(encoding="utf-8")
        self.assertIn('SEDSPRINTF_RS_MAX_STACK_PAYLOAD "4"', cmake)
        self.assertIn("RADIO_UART_MAX_PAYLOAD_SIZE    256U", radio)
        self.assertIn("can_bus_send_large", telemetry := (
            ROOT / "Core" / "Src" / "telemetry.c"
        ).read_text(encoding="utf-8"))

    def test_allocator_failure_exports_postmortem_snapshot(self):
        hooks = (ROOT / "Core" / "Src" / "telemetry_hooks.c").read_text(
            encoding="utf-8"
        )
        for symbol in (
            "g_telemetry_pool_low_water",
            "g_telemetry_alloc_failure_request",
            "g_telemetry_alloc_failure_available",
            "g_telemetry_alloc_failure_fragments",
        ):
            self.assertIn(symbol, hooks)


if __name__ == "__main__":
    unittest.main()
