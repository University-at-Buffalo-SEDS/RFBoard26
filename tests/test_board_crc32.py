"""Hardware CRC-32 hook: glue against a host model of the STM32 CRC unit."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
GLUE = ROOT / "Core/Src/board_crc32.c"
TEST = ROOT / "tests/native/board_crc32_test.c"

# Mirrors the hook block of SEDSnet's C-Headers/sedsnet.h.
SEDSNET_HOOK_API = """#include <stddef.h>
#include <stdint.h>
typedef uint32_t (*SedsCrc32Fn)(uint32_t crc, const uint8_t *data, size_t len);
void seds_set_crc32_fn(SedsCrc32Fn fn);
"""


class BoardCrc32Tests(unittest.TestCase):
    exe = None
    tmp = None

    @classmethod
    def build(cls, sanitize):
        directory = Path(cls.tmp.name)
        (directory / "sedsnet.h").write_text(SEDSNET_HOOK_API)
        exe = directory / f"board_crc32_{sanitize.replace(',', '_')}"
        subprocess.run([
            "cc", "-std=c11", "-D_GNU_SOURCE", "-O1", "-g", "-Wall", "-Wextra", "-Werror",
            f"-fsanitize={sanitize}", "-DBOARD_CRC32_HOST_MODEL",
            "-I", str(directory), "-I", str(ROOT / "Core/Inc"), "-I", str(ROOT / "tests/native"),
            str(GLUE), str(TEST), "-pthread", "-o", str(exe),
        ], check=True)
        return exe

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.exe = cls.build("address,undefined")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_case(self, name, exe=None):
        result = subprocess.run([str(exe or self.exe), name], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(f"PASS {name}", result.stdout)

    def test_model_matches_stm32_reference(self):
        self.run_case("test_model_matches_stm32_reference")

    def test_software_path(self):
        self.run_case("test_software_path")

    def test_install_and_vectors(self):
        self.run_case("test_install_and_vectors")

    def test_contention_falls_back_to_software(self):
        self.run_case("test_contention_falls_back_to_software")

    def test_failed_self_test_installs_nothing(self):
        self.run_case("test_failed_self_test_installs_nothing")

    def test_threads(self):
        self.run_case("test_threads")

    def test_threads_under_tsan(self):
        try:
            exe = self.build("thread")
        except subprocess.CalledProcessError:
            self.skipTest("ThreadSanitizer is unavailable")
        self.run_case("test_threads", exe)

    def test_installed_before_scheduler_and_router(self):
        main = (ROOT / "Core/Src/main.c").read_text()
        install = main.index("board_crc32_install();")
        self.assertLess(install, main.index("MX_ThreadX_Init();"))
        self.assertLess(main.index("/* USER CODE BEGIN SysInit */"), install)
        self.assertLess(install, main.index("/* USER CODE END SysInit */"))
        cmake = (ROOT / "CMakeLists.txt").read_text()
        self.assertIn("Core/Src/board_crc32.c", cmake)


if __name__ == "__main__":
    unittest.main()
