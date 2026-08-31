import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIXED_LAUNCHCORE = "5e5c9d52f91e53b0113aaee7355bc354118538a6"
UNSAFE_HANDOFFS = {
    "1ab6cd3dcddb7acaacb9dbfc16159f36f19363a8",
    "709474c68b83d259ba8657038340577ed4e8c6e4",
}


class LaunchCoreHandoffContract(unittest.TestCase):
    def test_bootloader_uses_stack_safe_application_handoff(self):
        cmake = (ROOT / "cmake/launchcore_stm32.cmake").read_text()
        self.assertIn(f"GIT_TAG {FIXED_LAUNCHCORE}", cmake)
        for unsafe_revision in UNSAFE_HANDOFFS:
            self.assertNotIn(unsafe_revision, cmake)


if __name__ == "__main__":
    unittest.main()
