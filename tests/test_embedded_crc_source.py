"""Exercise CMake preparation and Cargo resolution, including stale locks."""
import json
import shutil
import subprocess
import tempfile
import tomllib
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class EmbeddedCrcSourceTests(unittest.TestCase):
    def prepare(self, source, crc):
        result = subprocess.run([
            "cmake", f"-DSEDSNET_SOURCE_DIR={source}",
            f"-DSEDSNET_SCHEMA_FILE={ROOT / 'config/sedsnet.json'}",
            f"-DSEDSNET_CRC32_DIR={crc}",
            "-P", str(ROOT / "cmake/prepare_sedsnet.cmake"),
        ], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        return tomllib.loads((source / "Cargo.toml").read_text())

    def fixture(self, directory):
        source = directory / "sdk"
        (source / "C-Headers").mkdir(parents=True)
        (source / "C-Headers/sedsnet.h").write_text(
            "int seds_endpoint_identity();\nint seds_dtype_identity();\n")
        (source / "Cargo.toml").write_text(
            '[dependencies]\n'
            'crc32fast = { version = "1.5.2", default-features = false }\n'
            '[features]\nembedded = ["compression", "timesync", "discovery", '
            '"cryptography", "serde", "serde_json", "serde_json/alloc"]\n')
        return source

    def test_newer_registry_lock_cannot_replace_board_crc(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            source = self.fixture(directory)
            crc = ROOT / "third_party/embedded-crc32fast"
            manifest = self.prepare(source, crc)
            dependency = manifest["dependencies"]["crc32fast"]
            self.assertEqual(Path(dependency["path"]).resolve(), crc.resolve())
            self.assertEqual(dependency["version"], "1.5.2")
            # Resolve the prepared dependency with real Cargo, independently
            # of the minimal schema-feature fixture above. The obsolete lock
            # deliberately advertises a future, higher registry patch version.
            probe = directory / "probe"
            (probe / "src").mkdir(parents=True)
            (probe / "src/lib.rs").write_text("pub fn probe() {}\n")
            (probe / "Cargo.toml").write_text(
                '[package]\nname = "crc-override-probe"\nversion = "0.1.0"\n'
                'edition = "2021"\n[dependencies]\n'
                f'crc32fast = {{ version = "{dependency["version"]}", '
                f'path = {json.dumps(dependency["path"])}, default-features = false }}\n')
            (probe / "Cargo.lock").write_text(
                'version = 4\n[[package]]\nname = "crc-override-probe"\n'
                'version = "0.1.0"\ndependencies = ["crc32fast"]\n'
                '[[package]]\nname = "crc32fast"\nversion = "1.5.99"\n'
                'source = "registry+https://github.com/rust-lang/crates.io-index"\n')
            result = subprocess.run([
                "cargo", "metadata", "--offline", "--format-version", "1",
                "--manifest-path", str(probe / "Cargo.toml"),
            ], text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            packages = [p for p in json.loads(result.stdout)["packages"]
                        if p["name"] == "crc32fast"]
            self.assertEqual(len(packages), 1)
            self.assertIsNone(packages[0]["source"])
            self.assertEqual(Path(packages[0]["manifest_path"]).resolve(),
                             (crc / "Cargo.toml").resolve())

    def test_prepared_source_updates_both_paths_after_move(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            source = self.fixture(directory)
            first = directory / "first-crc"
            second = directory / "moved-crc"
            shutil.copytree(ROOT / "third_party/embedded-crc32fast", first)
            self.prepare(source, first)
            shutil.move(first, second)
            manifest = self.prepare(source, second)
            self.assertEqual(Path(manifest["dependencies"]["crc32fast"]["path"]), second)
            self.assertEqual(Path(manifest["patch"]["crates-io"]["crc32fast"]["path"]), second)
            before = (source / "Cargo.toml").read_text()
            self.prepare(source, second)
            self.assertEqual((source / "Cargo.toml").read_text(), before)


if __name__ == "__main__":
    unittest.main()
