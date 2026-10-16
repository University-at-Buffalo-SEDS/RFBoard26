"""LTO-renamed counters must still probe the exact firmware address."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
from sim import run_full

class LtoProbeResolutionTests(unittest.TestCase):
    def load(self, symbols):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "sim").mkdir()
            (root / "build/Test").mkdir(parents=True)
            (root / "build/Test/board.elf").touch()
            (root / "sim/board.json").write_text(json.dumps({
                "artifacts": {"elf": "build/Release/board.elf"},
                "execution": {"memory_probes": [{"name": "drops", "symbol": "g_drop", "maximum": 0}]},
            }))
            with mock.patch.object(run_full.shutil, "which", return_value="/tools/nm"), mock.patch.object(run_full.subprocess, "check_output", return_value=symbols):
                return run_full.load_layout_for_build(root, "Test")["execution"]["memory_probes"][0]

    def test_unique_lto_suffix_keeps_counter_name_and_limit(self):
        self.assertEqual(self.load("20000000 B g_drop.lto_priv.0\n"), {"name": "drops", "symbol": "g_drop.lto_priv.0", "maximum": 0})

    def test_exact_symbol_wins_over_other_translation_unit(self):
        self.assertEqual(self.load("20000000 B g_drop\n20000100 B g_drop.lto_priv.0\n")["symbol"], "g_drop")

    def test_ambiguous_and_noncompiler_suffixes_fail_closed(self):
        for symbols in ["20000000 B g_drop.lto_priv.0\n20000100 B g_drop.lto_priv.1\n", "20000000 B g_drop.lto_priv.unrelated\n"]:
            with self.subTest(symbols=symbols), self.assertRaisesRegex(RuntimeError, "g_drop"):
                self.load(symbols)

if __name__ == "__main__":
    unittest.main()
