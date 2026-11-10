"""Run the RFD modem check (Core/Src/rfd_modem.c) against a simulated modem.

Covers parse/plan/apply, the flight gate, a modem that does not answer,
link-wide registers that are not rolled out, the RFD ASYNC 4.03 register
layout (matched by ATI5 name) and the BOARD_HEALTH status block. Also checks
that the startup wiring keeps the check skippable and bounded.
"""

import json
import sys
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class RfdModemCheckTests(unittest.TestCase):
    def test_native_check_against_simulated_modem(self):
        with tempfile.TemporaryDirectory() as directory:
            executable = str(Path(directory) / "rfd_modem_test")
            subprocess.run(
                [
                    "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wconversion",
                    "-Wno-sign-conversion", "-fsanitize=address,undefined",
                    "-fno-omit-frame-pointer", "-I", str(ROOT / "Core/Inc"),
                    str(ROOT / "Core/Src/rfd_modem.c"),
                    str(ROOT / "tests/native/rfd_modem_test.c"), "-o", executable,
                ],
                check=True,
            )
            result = subprocess.run([executable], check=True, capture_output=True, text=True)
            self.assertGreaterEqual(result.stdout.count("ok "), 31, result.stdout)

    def test_constants_match_the_groundstation_check(self):
        source = (ROOT / "Core/Src/rfd_modem.c").read_text()
        # Same protocol timing as the GS check. The overall bound is RF's own:
        # RF's radio is deaf for the whole check, so it is capped at 16 s
        # (reboot and verify included) instead of the GS 40 s.
        for field, value in [("guard", 1100), ("plus_reply", 2500), ("command", 1000),
                             ("idle", 150), ("save", 2000), ("reboot", 4000),
                             ("overall", 16000)]:
            self.assertRegex(source, rf"\.{field} = {value}U")
        self.assertIn('"RTSCTS"', source)
        self.assertIn('SER_BRK_DETMS', source)

    def test_profile_is_the_groundstation_default(self):
        profile = (ROOT / "config/rfd-profile.json").read_bytes()
        modem = json.loads(profile)["modem"]
        self.assertEqual(
            (modem["max_data_bytes"], modem["max_retries"], modem["broadcast_retries"],
             modem["serial_break_ms"]), (3968, 10, 1, 10))
        # Byte-identical to the GS embedded default when a sibling checkout exists.
        sibling = ROOT.parent / "GroundStation26/backend/comms/rfd-profile.json"
        if sibling.is_file():
            self.assertEqual(profile, sibling.read_bytes())

    def test_usable_air_rate_fits_the_broadcast_copies(self):
        # Every broadcast packet holds the air for 1 + GLOBAL_RETRIES copies:
        # S5=10 left 64000/11 bit/s for 32000 bit/s of pacing (GS 707d9cd).
        import copy, subprocess, sys, tempfile
        profile = json.loads((ROOT / "config/rfd-profile.json").read_text())
        bad = copy.deepcopy(profile)
        bad["modem"]["broadcast_retries"] = 10
        for value, ok in ((profile, True), (bad, False)):
            with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
                json.dump(value, f)
            rc = subprocess.run([sys.executable, str(ROOT / "scripts/rfd_profile.py"), "--profile", f.name],
                                capture_output=True).returncode
            self.assertEqual(rc == 0, ok, value["modem"])

    def test_default_profile_paces_rf_at_the_modem_drain_rate(self):
        # The modem arbitrates the air (ASYNC, LBT, its own copies): RF pacing
        # only keeps the 4 KiB modem buffer from overflowing, so RF may use
        # the whole serial-to-air drain rate, not a fixed share of it.
        sys.path.insert(0, str(ROOT))
        from scripts import rfd_profile
        values = rfd_profile.cmake_values(ROOT / "config/rfd-profile.json")
        modem = json.loads((ROOT / "config/rfd-profile.json").read_text())["modem"]
        drain = modem["air_bps"] // (1 + modem["broadcast_retries"])
        self.assertEqual(values["RADIO_AIR_SHARE_PERCENT"], 100)
        self.assertLessEqual(values["RADIO_AIR_BIT_RATE_BPS"], drain)
        self.assertGreaterEqual(values["RADIO_AIR_BIT_RATE_BPS"], drain * 8 // 10)

    def test_heard_air_is_charged_to_rf_pacing_at_most_sixty_percent(self):
        # Every modem shares one half-duplex channel; RF paces its UART just
        # under one modem's drain rate. Air RF hears from GS and RRLS is
        # charged to RF's pacing (at most 60% of it) so both busy hosts stay
        # under the drain and neither modem's 4 KiB buffer overflows.
        radio = (ROOT / "Core/Src/radio.c").read_text()
        start = radio.index("#define RADIO_HEARD_AIR_MAX_PERCENT")
        end = radio.index("#endif", radio.index("static void radio_uart_note_heard_air(uint32_t bytes)\n{"))
        self.assertIn("radio_uart_note_heard_air(item.len);", radio)
        harness = """
#include <stdint.h>
#include <stdio.h>
#define RADIO_AIR_BIT_RATE_BPS 30000U
#define RADIO_AIR_SHARE_PERCENT 100U
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
static uint32_t g_now;
static uint32_t radio_now_ms(void) { return g_now; }
static volatile uint32_t g_tx_quiet_until_ms;
static volatile uint8_t g_tx_air_budget_active;
static uint32_t radio_uart_air_ms(uint16_t len) { return (uint32_t)(((uint64_t)len + 16U) * 8000U / 30000U); }
""" + radio[start:end] + """
int main(void) {
  /* One 200-byte packet heard: (200 + 16) * 8 / 30000 s = 57.6 ms. */
  g_now = 1000U; radio_uart_note_heard_air(200U);
  printf("%u\\n", (unsigned)(g_tx_quiet_until_ms - g_now));
  /* A neighbour using the whole channel for 10 s: RF is charged 60%. */
  g_tx_quiet_until_ms = 0U; g_tx_air_budget_active = 0U;
  uint32_t charged = 0U;
  for (g_now = 5000U; g_now < 15000U; g_now += 10U) {
    const uint32_t before = g_tx_air_budget_active && (int32_t)(g_tx_quiet_until_ms - g_now) > 0 ?
        g_tx_quiet_until_ms : g_now;
    radio_uart_note_heard_air(38U);
    charged += g_tx_quiet_until_ms - before;
  }
  printf("%u\\n", (unsigned)charged);
  return 0;
}
"""
        with tempfile.TemporaryDirectory() as d:
            exe = str(Path(d) / "heard")
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-x", "c", "-", "-o", exe],
                           input=harness, text=True, check=True)
            one, saturated = [int(v) for v in subprocess.run([exe], capture_output=True, text=True,
                                                             check=True).stdout.split()]
        self.assertIn(one, (57, 58))
        self.assertGreaterEqual(saturated, 5_500)
        self.assertLessEqual(saturated, 6_200)

    def test_pacing_follows_the_modems_actual_global_retries(self):
        # Bench: the modem still read S5=10 (eleven air copies) while the
        # profile paces for S5=1; RF must pace for what the air carries.
        radio = (ROOT / "Core/Src/radio.c").read_text()
        start = radio.index("static uint32_t radio_uart_air_ms(uint16_t len)\n{")
        end = radio.index("\n}\n", start) + 3
        harness = """
#include <stdint.h>
#include <stdio.h>
#define RFBOARD_LORA_SCHEDULER 0
#define RADIO_AIR_BIT_RATE_BPS 30000U
#define RADIO_AIR_SHARE_PERCENT 100U
#define RADIO_AIR_FRAME_OVERHEAD_BYTES 16U
#define RFD_PROFILE_AIR_BPS 64000U
volatile uint32_t g_rfd_modem_status[3];
""" + radio[start:end] + """
int main(void) {
  /* 984 + 16 bytes: 266 ms at 30000 bit/s, 1463 ms at 5469 bit/s. */
  printf("%u ", (unsigned)radio_uart_air_ms(984U));           /* no readback */
  g_rfd_modem_status[0] = 1U | (1U << 24);                    /* S5=1 */
  printf("%u ", (unsigned)radio_uart_air_ms(984U));
  g_rfd_modem_status[0] = 1U | (10U << 24);                   /* S5=10 */
  printf("%u ", (unsigned)radio_uart_air_ms(984U));
  g_rfd_modem_status[0] = 1U | (0xFFU << 24);                 /* unread */
  printf("%u", (unsigned)radio_uart_air_ms(984U));
  return 0;
}
"""
        with tempfile.TemporaryDirectory() as d:
            exe = str(Path(d) / "paced")
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-x", "c", "-", "-o", exe],
                           input=harness, text=True, check=True)
            out = subprocess.run([exe], capture_output=True, text=True, check=True).stdout.split()
        self.assertEqual([int(v) for v in out], [267, 267, 1463, 267])

    def test_default_build_applies_the_checked_in_profile(self):
        # The checked-in profile is the deployed configuration of all three
        # modems: a plain Release build writes its link-wide registers
        # (S5=1) with no --rfd-profile-rolled-out flag.
        cmake = (ROOT / "CMakeLists.txt").read_text()
        self.assertRegex(cmake, r'RFD_PROFILE_FILE}" STREQUAL "\$\{CMAKE_CURRENT_SOURCE_DIR}/config/rfd-profile.json"')
        self.assertIn("RFD_PROFILE_ROLLED_OUT=$<BOOL:${RFD_PROFILE_LINK_WRITES}>", cmake)

    def test_startup_wiring_is_gated_and_skippable(self):
        port = (ROOT / "Core/Src/rfd_modem_port.c").read_text()
        thread = (ROOT / "Core/Src/telemetry_thread.c").read_text()
        cmake = (ROOT / "CMakeLists.txt").read_text()
        # Once per boot, before CAN and the router start; never at runtime.
        boot = thread.index("rfd_modem_port_boot(telemetry_boot_service);")
        self.assertLess(boot, thread.index("can_bus_init(&hfdcan2);"))
        self.assertLess(boot, thread.index("init_telemetry_router()"))
        self.assertIn("rfd_modem_port_poll(NULL);", thread)
        self.assertEqual(port.count("radio_uart_set_modem_hold(1U)"), 1)
        # Simulator builds default to skip; operational builds to fix.
        self.assertRegex(cmake, r"SEDS_FIRMWARE_SIM_TEST\)\s*\n\s*set\(RFD_MODEM_CHECK_DEFAULT SKIP\)")
        self.assertIn("RFD_PROFILE_ROLLED_OUT", cmake)
        self.assertTrue(re.search(r"option\(RFD_PROFILE_ROLLED_OUT [^\n]* OFF\)", cmake))


if __name__ == "__main__":
    unittest.main()
