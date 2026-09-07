import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class NetworkSchemaContractTests(unittest.TestCase):
    def test_umbilical_command_ack_is_reliable_and_unordered(self):
        schema = json.loads((ROOT / "config" / "sedsnet.json").read_text(encoding="utf-8"))
        status = next(item for item in schema["types"] if item["name"] == "UMBILICAL_STATUS")
        self.assertTrue(status["reliable"])
        self.assertEqual(status["reliable_mode"], "Unordered")


if __name__ == "__main__":
    unittest.main()
