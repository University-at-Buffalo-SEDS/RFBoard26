import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class CanHardwareContract(unittest.TestCase):
    def test_matches_the_avionics_fd_bus(self):
        source = (ROOT / "Core/Src/main.c").read_text()
        ioc = (ROOT / "RFBoard26.ioc").read_text()

        self.assertIn("hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS", source)
        self.assertIn("hfdcan2.Init.AutoRetransmission = ENABLE", source)
        self.assertIn("hfdcan2.Init.NominalPrescaler = 10", source)
        self.assertIn("hfdcan2.Init.NominalTimeSeg1 = 13", source)
        self.assertIn("hfdcan2.Init.NominalTimeSeg2 = 3", source)
        self.assertIn("FDCAN2.CalculateBaudRateNominal=999999", ioc)
        self.assertIn("FDCAN2.AutoRetransmission=ENABLE", ioc)
        can = (ROOT / "Core/Src/can_bus.c").read_text()
        self.assertIn("CAN_BUS_TX_ENQUEUE_TIMEOUT_MS 5U", can)
        self.assertIn("HAL_FDCAN_AbortTxRequest", can)
        self.assertNotIn("< (uint32_t)frag_cnt", can)


if __name__ == "__main__":
    unittest.main()
