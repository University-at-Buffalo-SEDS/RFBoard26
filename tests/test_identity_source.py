import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class IdentitySourceTests(unittest.TestCase):
    def prepare(self, header):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "sdk"
            (source / "C-Headers").mkdir(parents=True)
            (source / "C-Headers/sedsnet.h").write_text(header)
            (source / "Cargo.toml").write_text(
                '[features]\nembedded = ["compression", "timesync", "discovery", '
                '"cryptography", "serde", "serde_json", "serde_json/alloc"]\n')
            result = subprocess.run(["cmake", f"-DSEDSNET_SOURCE_DIR={source}",
                f"-DSEDSNET_SCHEMA_FILE={ROOT / 'config/sedsnet.json'}",
                f"-DSEDSNET_CRC32_DIR={ROOT / 'third_party/embedded-crc32fast'}",
                "-P", str(ROOT / "cmake/prepare_sedsnet.cmake")],
                text=True, capture_output=True)
            return result, (source / "telemetry_config.json").exists()

    def test_legacy_offline_source_is_rejected_before_schema_mutation(self):
        result, copied = self.prepare("typedef unsigned int SedsDataType;\n")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("lacks canonical schema identities", result.stderr)
        self.assertFalse(copied)

    def test_compatible_offline_source_is_prepared_without_version_pin(self):
        result, copied = self.prepare(
            "int seds_endpoint_identity(const char *, unsigned, unsigned *);\n"
            "int seds_dtype_identity(const char *, unsigned, unsigned *);\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(copied)


if __name__ == "__main__":
    unittest.main()
