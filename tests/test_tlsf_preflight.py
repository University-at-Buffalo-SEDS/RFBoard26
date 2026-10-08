"""Read-only allocator admission must agree with real allocation under churn."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
class PreflightTests(unittest.TestCase):
    def test_fragmented_preflight_matches_memalign_without_mutation(self):
        with tempfile.TemporaryDirectory() as tmp:
            exe=Path(tmp)/"preflight"
            subprocess.run(["cc","-std=c11","-fsanitize=address,undefined","-I",str(ROOT/"third_party/tlsf"),str(ROOT/"tests/native/tlsf_preflight.c"),str(ROOT/"third_party/tlsf/tlsf.c"),"-o",str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
