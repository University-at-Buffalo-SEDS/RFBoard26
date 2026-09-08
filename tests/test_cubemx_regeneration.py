import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class CubeMxRegenerationContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ioc = next(ROOT.glob("*.ioc")).read_text(encoding="utf-8")
        cls.rtos = (ROOT / "AZURE_RTOS/App/app_azure_rtos_config.h").read_text(
            encoding="utf-8"
        )
        cls.cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    def test_cubemx_preserves_user_code_and_cmake_project(self):
        self.assertIn("ProjectManager.KeepUserCode=true", self.ioc)
        self.assertIn("ProjectManager.TargetToolchain=CMake", self.ioc)
        self.assertIn("add_subdirectory(cmake/stm32cubemx)", self.cmake)

    def test_ioc_is_source_of_truth_for_generated_rtos_pools(self):
        for macro in ("TX_APP_MEM_POOL_SIZE", "UX_DEVICE_APP_MEM_POOL_SIZE"):
            generated = re.search(
                rf"^#define\\s+{macro}\\s+(\\d+)", self.rtos, re.MULTILINE
            )
            if generated is None:
                continue
            configured = re.search(
                rf"^[^=]*\\.{macro}=(\\d+)$", self.ioc, re.MULTILINE
            )
            self.assertIsNotNone(configured, f"{macro} is absent from the .ioc")
            self.assertEqual(
                int(generated.group(1)),
                int(configured.group(1)),
                f"CubeMX would regenerate a different {macro}",
            )

    def test_board_owned_build_reconnects_external_core_libraries(self):
        self.assertRegex(self.cmake.lower(), r"sedsnet")
        self.assertRegex(self.cmake.lower(), r"launchcore")


if __name__ == "__main__":
    unittest.main()

