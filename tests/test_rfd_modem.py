"""Run the RFD modem check (Core/Src/rfd_modem.c) against a simulated modem.

Covers parse/plan/apply, the flight gate, a modem that does not answer,
link-wide registers that are not rolled out, the RFD ASYNC 4.03 register
layout (matched by ATI5 name) and the BOARD_HEALTH status block. Also checks
that the startup wiring keeps the check skippable and bounded.
"""

import json
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
            self.assertGreaterEqual(result.stdout.count("ok "), 26, result.stdout)

    def test_constants_match_the_groundstation_check(self):
        source = (ROOT / "Core/Src/rfd_modem.c").read_text()
        for field, value in [("guard", 1100), ("plus_reply", 2500), ("command", 1000),
                             ("idle", 150), ("save", 2000), ("reboot", 4000),
                             ("overall", 40000)]:
            self.assertRegex(source, rf"\.{field} = {value}U")
        self.assertIn('"RTSCTS"', source)
        self.assertIn('SER_BRK_DETMS', source)

    def test_profile_is_the_groundstation_default(self):
        profile = (ROOT / "config/rfd-profile.json").read_bytes()
        modem = json.loads(profile)["modem"]
        self.assertEqual(
            (modem["max_data_bytes"], modem["max_retries"], modem["broadcast_retries"],
             modem["serial_break_ms"]), (3968, 10, 10, 50))
        # Byte-identical to the GS embedded default when a sibling checkout exists.
        sibling = ROOT.parent / "GroundStation26/backend/comms/rfd-profile.json"
        if sibling.is_file():
            self.assertEqual(profile, sibling.read_bytes())

    def test_startup_wiring_is_gated_and_skippable(self):
        port = (ROOT / "Core/Src/rfd_modem_port.c").read_text()
        thread = (ROOT / "Core/Src/telemetry_thread.c").read_text()
        cmake = (ROOT / "CMakeLists.txt").read_text()
        self.assertIn("rfd_modem_port_poll(telemetry_service_without_radio)", thread)
        # At most one AT&W per MCU boot.
        self.assertIn("g_rfd_saved_this_boot", port)
        self.assertIn("rfd_gate_decide", port)
        # Simulator builds default to skip; operational builds to fix.
        self.assertRegex(cmake, r"SEDS_FIRMWARE_SIM_TEST\)\s*\n\s*set\(RFD_MODEM_CHECK_DEFAULT SKIP\)")
        self.assertIn("RFD_PROFILE_ROLLED_OUT", cmake)
        self.assertTrue(re.search(r"option\(RFD_PROFILE_ROLLED_OUT [^\n]* OFF\)", cmake))


if __name__ == "__main__":
    unittest.main()
