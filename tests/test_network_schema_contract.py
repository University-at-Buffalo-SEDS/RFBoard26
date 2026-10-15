import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class NetworkSchemaContractTests(unittest.TestCase):
    def test_every_type_has_one_explicit_reliability_value(self):
        schema = json.loads((ROOT / "config/sedsnet.json").read_text())
        for item in schema["types"]:
            self.assertNotIn("reliable_mode", item, item["name"])
            self.assertIn(item["reliable"], ("Off", "Ordered", "Unordered"), item["name"])
        temperatures = [item for item in schema["types"] if item["name"] == "DAQ_ADC_TEMPERATURE"]
        for item in temperatures:
            self.assertEqual(item["reliable"], "Off")

    def test_umbilical_command_ack_is_reliable_and_ordered(self):
        schema = json.loads((ROOT / "config" / "sedsnet.json").read_text(encoding="utf-8"))
        status = next(item for item in schema["types"] if item["name"] == "UMBILICAL_STATUS")
        self.assertEqual(status["reliable"], "Ordered")


class StableSchemaIdentityTests(unittest.TestCase):
    def test_c_header_uses_canonical_name_identities(self):
        import re
        schema = json.loads((ROOT / "config/sedsnet.json").read_text())
        header = (ROOT / "Core/Inc/sedsnet_config.h").read_text()
        legacy_endpoints = {"SD_CARD": 100, "RADIO": 101, "SOFTWARE_BUS": 102}
        legacy_types = {name: 100 + index for index, name in enumerate([
            "GPS_DATA", "IMU_DATA", "BATTERY_STATUS", "SYSTEM_STATUS",
            "BAROMETER_DATA", "MESSAGE_DATA", "HEARTBEAT", "IPC_MESSAGE"])}
        for kind, collection, namespace, legacy in [
            ("EP", "endpoints", 0x4550, legacy_endpoints),
            ("DT", "types", 0x5459, legacy_types),
        ]:
            names = {entry["name"] for entry in schema[collection]}
            for name, numeric in re.findall(
                rf"#define SEDS_{kind}_(\w+) \(\(SedsData\w+\)(\d+)U\)", header
            ):
                self.assertIn(name, names)
                value = 0x811c9dc5 ^ namespace
                for byte in name.encode("utf-8"):
                    value = ((value ^ byte) * 0x01000193) & 0xffffffff
                expected = legacy.get(name, 0x10000 | (value & 0x0fffffff))
                self.assertEqual(int(numeric), expected, name)


if __name__ == "__main__":
    unittest.main()
