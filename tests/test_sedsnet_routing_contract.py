import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class SedsnetRoutingContract(unittest.TestCase):
    def test_firmware_does_not_install_manual_routes(self):
        sources = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (ROOT / "Core" / "Src").glob("*.c")
        )
        self.assertNotIn("seds_router_set_route", sources)
        self.assertNotIn("seds_router_set_typed_route", sources)


if __name__ == "__main__":
    unittest.main()
