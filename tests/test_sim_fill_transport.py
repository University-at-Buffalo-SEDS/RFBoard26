"""Qualification must exercise the same framed transports as deployment."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location("fill_transport_runner", Path(__file__).resolve().parents[1] / "sim/run_full.py")
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)

class FillTransportQualificationTests(unittest.TestCase):
    def test_default_discovery_and_explicit_legacy_endpoints(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(RUNNER.simulated_fill_link(), {"link": "fill_pico", "env": "GS_FILL_PICO_DISCOVERY_PORT", "transport": "pico_fi_discovery_to_uart"})
        with patch.dict(os.environ, {"SEDS_FIRMWARE_SIM_FILL_TRANSPORT": "tcp"}):
            self.assertEqual(RUNNER.simulated_fill_link(), {"link": "fill_pico", "env": "GS_FILL_PICO_TCP_ADDRESS", "transport": "pico_fi_tcp_to_uart"})
        with patch.dict(os.environ, {"SEDS_FIRMWARE_SIM_FILL_TRANSPORT": "i2c"}):
            self.assertEqual(RUNNER.simulated_fill_link(), {"link": "fill_pico", "env": "GS_SIMULATED_I2C_SOCKET", "transport": "pico_fi_i2c_to_uart"})

    def test_invalid_transport_fails_before_starting_network(self):
        with patch.dict(os.environ, {"SEDS_FIRMWARE_SIM_FILL_TRANSPORT": "typo"}):
            with self.assertRaisesRegex(RuntimeError, "must be discovery, tcp or i2c"):
                RUNNER.simulated_fill_link()

    def test_generated_config_uses_discovery_without_modifying_live_settings(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.dict(os.environ, {}, clear=True):
                RUNNER.write_network_comms_config(root)
            cfg = json.loads((root / "comms-v2.json").read_text())
            self.assertEqual(cfg["fill_box"], {"interface":"pico_fi_tcp", "mode":"server", "address":"0.0.0.0:0", "discovery":True})
            self.assertEqual(cfg["av_bay"]["baud_rate"], 115200)
            self.assertEqual(list(root.iterdir()), [root / "comms-v2.json"])
