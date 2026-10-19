"""Real preprocessing must exclude test diagnostics when SIM is zero/off."""
from pathlib import Path
import re
import subprocess
import unittest
ROOT = Path(__file__).resolve().parents[1]

class SimDiagnosticsGateTests(unittest.TestCase):
    def test_numeric_sim_flag_excludes_production_diagnostics(self):
        source = (ROOT / "Core/Src/telemetry.c").read_text()
        # Preprocess the real translation unit; types/includes are irrelevant
        # to conditional inclusion and need no mocked STM32 implementation.
        source = re.sub(r"^\s*#\s*include[^\n]*", "", source, flags=re.MULTILINE)
        for value, enabled in [(None, False), ("0", False), ("1", True)]:
            with self.subTest(value=value):
                args = ["cc", "-E", "-P", "-x", "c", "-",
                        "-DTELEMETRY_ENABLED=1", "-DTELEMETRY_USE_TLSF=1",
                        "-DTX_TIMER_TICKS_PER_SECOND=1000"]
                if value is not None:
                    args.append("-DSEDS_FIRMWARE_SIM_TEST=" + value)
                result = subprocess.run(args, input=source, text=True,
                                        capture_output=True, check=True)
                for symbol in ["g_rf_memory_counters", "g_rf_memory_snapshot_tick"]:
                    self.assertEqual(symbol in result.stdout, enabled, symbol)
                self.assertEqual("g_sim_radio_egress_peer_mask |=" in result.stdout, enabled)
                self.assertEqual("seds_router_copy_memory_counters" in result.stdout,
                                 enabled)
