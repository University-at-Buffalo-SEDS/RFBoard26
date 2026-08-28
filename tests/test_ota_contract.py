import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def macro(name: str) -> int:
    text = (ROOT / "Bootloader/board_config.h").read_text()
    match = re.search(rf"#define\s+{name}\s+(0x[0-9A-Fa-f]+|[0-9]+)u?", text)
    if not match:
        raise AssertionError(f"missing {name}")
    return int(match.group(1), 0)


class OtaContractTests(unittest.TestCase):
    def test_flash_regions_are_aligned_and_non_overlapping(self):
        erase = macro("LAUNCHCORE_FLASH_ERASE_SIZE")
        slot_base = macro("BOARD_SLOT_A_BASE")
        slot_size = macro("BOARD_SLOT_A_SIZE")
        delta_base = macro("BOARD_DELTA_BASE")
        delta_size = macro("BOARD_DELTA_SIZE")
        metadata = macro("BOARD_METADATA0_BASE")
        self.assertEqual(slot_base + slot_size, delta_base)
        self.assertLessEqual(delta_base + delta_size, metadata)
        for value in (slot_base, slot_size, delta_base, delta_size, metadata):
            self.assertEqual(value % erase, 0)

    def test_stream_chunk_matches_flash_and_sedsnet_limits(self):
        source = (ROOT / "Core/Src/ota_stream.c").read_text()
        header = (ROOT / "Core/Inc/ota_stream.h").read_text()
        alignment = macro("BOARD_FLASH_WRITE_ALIGNMENT")
        chunk = int(re.search(r"OTA_STREAM_MAX_CHUNK\s+([0-9]+)U", header).group(1))
        cmake = (ROOT / "CMakeLists.txt").read_text()
        payload = int(re.search(r'SEDSNET_MAX_STACK_PAYLOAD "([0-9]+)"', cmake).group(1))
        self.assertEqual(chunk % alignment, 0)
        self.assertLessEqual(chunk + 5, payload)
        self.assertIn("seds_router_bind_p2p_stream_port", source)
        self.assertIn("launchcore_delta_update_finish", source)
        self.assertIn("launchcore_confirm_boot", source)

    def test_application_linker_stops_before_delta_partition(self):
        scripts = list(ROOT.glob("STM32*_FLASH.ld"))
        self.assertEqual(len(scripts), 1)
        text = scripts[0].read_text()
        match = re.search(
            r"(?:FLASH|ROM)\s*\(rx\)\s*:\s*ORIGIN\s*=\s*(0x[0-9A-Fa-f]+),"
            r"\s*LENGTH\s*=\s*(0x[0-9A-Fa-f]+)",
            text,
        )
        self.assertIsNotNone(match)
        origin, length = (int(value, 0) for value in match.groups())
        self.assertEqual(origin, macro("BOARD_VECTOR_TABLE"))
        self.assertEqual(origin + length, macro("BOARD_DELTA_BASE"))

    def test_vector_table_and_packaged_header_are_cortex_m_aligned(self):
        slot_base = macro("BOARD_SLOT_A_BASE")
        vector_table = macro("BOARD_VECTOR_TABLE")
        self.assertEqual(vector_table % 0x200, 0)
        self.assertEqual(vector_table - slot_base, 0x200)

        cmake = (ROOT / "cmake/launchcore_stm32.cmake").read_text()
        self.assertRegex(cmake, r"--header-size\s+0x200")

        platform = (ROOT / "Bootloader/platform.c").read_text()
        self.assertIn("(vector_table_addr & 0x1FFu) == 0u", platform)

    def test_bootloader_exposes_led_boot_state(self):
        platform = (ROOT / "Bootloader/platform.c").read_text()
        self.assertIn("status_led_init();", platform)
        self.assertIn("BOOT_STATUS_BLUE_PIN", platform)
        self.assertIn("BOOT_STATUS_GREEN_PIN", platform)


if __name__ == "__main__":
    unittest.main()
