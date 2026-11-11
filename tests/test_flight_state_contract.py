"""Network FLIGHT_STATE contract, set (a): what this board accepts and maps.

Only 0 Startup, 1 Idle, 2 Fill, 6 Armed, 7 Launch, 13 Landed, 15 Aborted and
16 InFlight are ever on the wire. A retired value from older firmware maps to
the nearest network value on receipt (3..5 Fill, 8..12 InFlight, 14 Landed).
"""
import json
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SET_A = {0: "Startup", 1: "Idle", 2: "Fill", 6: "Armed", 7: "Launch",
         13: "Landed", 15: "Aborted", 16: "InFlight"}
RETIRED = {3: 2, 4: 2, 5: 2, 8: 16, 9: 16, 10: 16, 11: 16, 12: 16, 14: 13}


def strip_comments(text):
    return re.sub(r"//[^\n]*", "", re.sub(r"/\*.*?\*/", "", text, flags=re.S))


class FlightStateContractTests(unittest.TestCase):
    def test_cache_accepts_in_flight(self):
        source = strip_comments((ROOT / "Core/Src/flight_state_cache.c").read_text())
        bound = re.search(r"#define\s+FLIGHT_STATE_MAX_VALUE\s+(\d+)", source)
        self.assertIsNotNone(bound)
        self.assertGreaterEqual(int(bound.group(1)), 16)
        self.assertIn("flight_state_cache_from_wire(packet->payload[0])", source)

    def test_state_enum_names_every_value_outside_set_a_retired(self):
        header = ROOT / "Core/Inc/thread_comm.h"
        if not header.exists() or "_FLIGHT_STATE_STARTUP" not in header.read_text():
            self.skipTest("this board keeps no flight-state enum")
        text = strip_comments(header.read_text())
        members = {name: int(value) for name, value in
                   re.findall(r"\b([A-Z]+_FLIGHT_STATE_\w+)\s*=\s*(\d+)", text)}
        self.assertIn(16, members.values())
        for name, value in members.items():
            if value not in SET_A:
                self.assertIn("RETIRED", name)
        self.assertFalse([name for name in members if "PREFILL" in name])

    def test_schema_documents_set_a(self):
        schema = json.loads((ROOT / "config/sedsnet.json").read_text())
        entry = next(t for t in schema["types"] if t["name"] == "FLIGHT_STATE")
        self.assertEqual(entry["element"], {"kind": "Static", "data_type": "UInt8", "count": 1})
        listed = [int(v) for v in re.findall(r"\b(\d+) [A-Z]", entry["doc"].split(":", 1)[1])]
        self.assertEqual(sorted(listed), sorted(SET_A))

    @unittest.skipUnless(shutil.which("cc"), "native C compiler unavailable")
    def test_every_received_value_maps_onto_set_a(self):
        expected = []
        for raw in range(256):
            if raw in SET_A:
                expected.append((raw, raw, 0))
            elif raw in RETIRED:
                expected.append((raw, RETIRED[raw], 1))
            else:
                expected.append((raw, 0xFF, 0))
        table = ",".join("{%d,%d,%d}" % row for row in expected)
        code = r'''
#include <assert.h>
#include <stdint.h>
#include "flight_state_cache.h"
static const uint8_t table[256][3]={%s};
int main(void){
 for(unsigned i=0;i<256;i++){uint8_t retired=9;
  assert(flight_state_from_wire(table[i][0],&retired)==table[i][1]&&retired==table[i][2]);
  assert(flight_state_from_wire(table[i][0],0)==table[i][1]);}
 return 0;}
''' % table
        with tempfile.TemporaryDirectory(prefix="flight-state-contract-") as directory:
            work = Path(directory)
            (work / "test.c").write_text(code)
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "Core/Inc"),
                            str(work / "test.c"), "-o", str(work / "test")], check=True)
            subprocess.run([str(work / "test")], check=True, timeout=10)


if __name__ == "__main__":
    unittest.main()
