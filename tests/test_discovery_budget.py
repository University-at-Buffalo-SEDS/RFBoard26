"""Exercise actual embedded SEDSnet discovery with the two exhausted RF limits.

Uses the fetched source without production modifications; host rlib linkage
allows a single-threaded harness with the standard allocator. Host metadata is
larger than ARM metadata, so this tests admission rather than ARM byte totals.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]
class BootstrapTests(unittest.TestCase):
    def test_bootstrap_budget_and_fixed_slots(self):
        cache = ROOT / "build/Release/CMakeCache.txt"
        if not cache.exists():
            self.skipTest("configure Release to fetch SEDSnet first")
        entry = next((line for line in cache.read_text().splitlines() if line.startswith("SEDSNET_DIR:PATH=")), None)
        if entry is None:
            self.skipTest("SEDSNET_DIR is not configured")
        source = Path(entry.split("=", 1)[1])
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            lib = work / "library"
            lib.mkdir()
            for name in ("src", "sedsnet_macros", "Cargo.toml", "Cargo.lock", "build.rs", "telemetry_config.json"):
                item = source / name
                if item.is_dir():
                    shutil.copytree(item, lib / name, ignore=shutil.ignore_patterns("target", ".git"))
                elif item.exists():
                    shutil.copy2(item, lib / name)
            manifest = lib / "Cargo.toml"
            text = manifest.read_text()
            import re
            text = re.sub(r'crate-type\s*=\s*\[[^\]]*\]', 'crate-type = ["rlib"]', text)
            manifest.write_text(text)
            (work / "src").mkdir()
            shutil.copy2(ROOT / "tests/native/discovery_budget.rs", work / "src/main.rs")
            (work / "Cargo.toml").write_text('[package]\nname="rf-bootstrap-regression"\nversion="0.1.0"\nedition="2024"\n[dependencies]\nsedsnet={package="SEDSnet",path="library",default-features=false,features=["embedded"]}\n')
            subprocess.run(["cargo", "run", "--offline", "--quiet"], cwd=work, check=True, timeout=180)
