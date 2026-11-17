"""The firmware requests SEDSnet's embedded zstd size options before the SDK is configured."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]
CMAKE = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
# The release link uses -flto, so zstd also joins that link as LTO objects.
RELEASE_LINK_USES_LTO = True


def forced(name: str) -> str | None:
    match = re.search(rf"set\({name} (ON|OFF) CACHE BOOL\s+\"[^\"]*\" FORCE\)", CMAKE)
    return match.group(1) if match else None


def sdk_configured_at() -> int:
    match = re.search(r"include\(cmake/sedsnet_fetch\.cmake\)|FetchContent_MakeAvailable\(sedsnet\)", CMAKE)
    assert match, "SEDSnet is not configured from CMakeLists.txt"
    return match.start()


class SedsnetZstdOptionTests(unittest.TestCase):
    def test_lean_zstd_is_requested_before_the_sdk_is_configured(self):
        self.assertEqual(forced("SEDSNET_ZSTD_LEAN"), "ON")
        self.assertLess(CMAKE.index("set(SEDSNET_ZSTD_LEAN"), sdk_configured_at())

    def test_zstd_lto_follows_the_release_link(self):
        if RELEASE_LINK_USES_LTO:
            self.assertEqual(forced("SEDSNET_ZSTD_LTO"), "ON")
            self.assertLess(CMAKE.index("set(SEDSNET_ZSTD_LTO"), sdk_configured_at())
        else:
            self.assertIsNone(forced("SEDSNET_ZSTD_LTO"))


if __name__ == "__main__":
    unittest.main()
