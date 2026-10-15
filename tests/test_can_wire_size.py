"""Check production CAN fragmentation against the existing receiver stride."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class CanWireSizeTests(unittest.TestCase):
    def test_short_single_fragment_and_legacy_multi_fragment_roundtrip(self):
        source = (ROOT / "Core/Src/can_bus.c").read_text()
        start = source.index("HAL_StatusTypeDef can_bus_send_large(")
        opening = source.index("{", start)
        depth = 1
        end = opening + 1
        while depth:
            depth += (source[end] == "{") - (source[end] == "}")
            end += 1
        function = source[start:end]
        hs = re.search(r"typedef struct\s+__attribute__\(\(packed\)\)\s*\{", source).start()
        he = source.index("} can_bus_frag_hdr_t;", hs) + len("} can_bus_frag_hdr_t;")
        header = source[hs:he]
        code = r"""
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_BUSY 2
#define CAN_BUS_FRAG_MAGIC 0x4453U
#define CAN_BUS_NODE_ID 7U
#define CAN_BUS_FRAG_WIRE_LEN 64U
#define CAN_BUS_FRAG_F_FIRST 1U
#define CAN_BUS_FRAG_F_LAST 2U
static void *g_hfdcan = (void *)1;
static unsigned g_can_tx_service_stage;
static uint8_t received[256];
static size_t expected, stride, frames, first_wire;
static int busy;
static HAL_StatusTypeDef can_bus_send_bytes(const uint8_t *data, size_t used, uint32_t id) {
    if (busy) return HAL_BUSY;
    assert(id == 0x107U && used <= 64 && used >= 9);
    static const unsigned dlc[] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    size_t wire = 0;
    for (unsigned i=0; i<16; i++) if (dlc[i]>=used) {wire=dlc[i];break;}
    assert(wire && data[5] && data[4] == frames);
    size_t total = data[7] | ((size_t)data[8] << 8);
    assert(total == expected);
    if (!frames) {stride=wire-9;first_wire=wire;}
    size_t offset = data[4]*stride, take=wire-9;
    assert(offset<total);
    if (take>total-offset) take=total-offset;
    memcpy(received+offset,data+9,take);
    if (data[5]>1) assert(wire==64);
    frames++;
    return HAL_OK;
}
""" + header + function + r"""
int main(void) {
    assert(sizeof(can_bus_frag_hdr_t)==9);
    uint8_t payload[128];for (unsigned i=0;i<128;i++) payload[i]=(uint8_t)(i*7);
    for (size_t len=1;len<=128;len++) {
        expected=len;frames=0;memset(received,0xCC,sizeof(received));
        assert(can_bus_send_large(payload,len,0x107)==HAL_OK);
        assert(frames==(len+54)/55 && memcmp(received,payload,len)==0);
        if (len<=23) assert(first_wire<=32);
        if (len>55) assert(first_wire==64);
    }
    busy=1;frames=0;expected=20;
    assert(can_bus_send_large(payload,20,0x107)==HAL_BUSY && !frames);
    assert(can_bus_send_large(NULL,20,0x107)==HAL_ERROR);
    assert(can_bus_send_large(payload,0,0x107)==HAL_ERROR);
    (void)g_can_tx_service_stage;
}
"""
        with tempfile.TemporaryDirectory() as directory:
            c=Path(directory)/"wire.c"; exe=Path(directory)/"wire"
            c.write_text(code)
            subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror","-fsanitize=address,undefined",str(c),"-o",str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
