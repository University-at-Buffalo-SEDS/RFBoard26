"""When the startup RFD modem check may take the radio.

The check holds the radio (no TX, RX ignored) for 3 to 14 s. In the faithful
sim (radio-tx-storm run fix5-rfprobe-a2) RF's last SEDSnet frame left at
27.738 s, the station's FLIGHT_STATE=1 write arrived at 27.752 s, and the
next telemetry pass took the hold: "+++" at 29.03 s, AT..ATZ to 30.75 s,
"+++" again at 36.06 s for the post-save verify. The write's ReliableAck
never left RF and every station retry met a deaf radio.

These tests compile the real port (rfd_modem_port.c with rfd_modem.c) and
the real radio hold admission against stubs.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

STUBS = {
    "tx_api.h": "#pragma once\ntypedef unsigned long ULONG;\nULONG tx_time_get(void);\n"
                "#define TX_TIMER_TICKS_PER_SECOND 1000U\n",
    "radio.h": "#pragma once\n#include <stddef.h>\n#include <stdint.h>\n"
               "uint8_t radio_uart_set_modem_hold(uint8_t on);\n"
               "int radio_uart_modem_write(const uint8_t *bytes, size_t len);\n"
               "int radio_uart_modem_read(uint8_t *buf, size_t cap);\n",
    "persistent_store.h": "#pragma once\n#include <stddef.h>\n#include <stdint.h>\n"
                          "typedef int launchcore_persist_status_t;\n"
                          "#define LAUNCHCORE_PERSIST_OK 0\n#define LAUNCHCORE_PERSIST_NOT_FOUND 1\n"
                          "launchcore_persist_status_t persistent_store_get(uint32_t,void*,size_t*);\n"
                          "launchcore_persist_status_t persistent_store_set(uint32_t,const void*,size_t);\n",
    "board_config.h": "#pragma once\n#define BOARD_SLOT_A_BASE 0x08004000u\n",
    "launchcore/image.h": "#pragma once\n#include <stdint.h>\n"
                          "typedef struct { uint32_t magic, sizes, image_size, image_crc32; }"
                          " launchcore_image_header_t;\n",
}

PORT_HARNESS = r'''
#include "rfd_modem.h"
#include "rfd_modem_port.h"
#include "persistent_store.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
volatile uint32_t g_flight_state_cache_updates, g_radio_rx_frames_ok;
extern volatile uint32_t g_rfd_modem_status[3];
static uint32_t image_id = 1U;
uint32_t rfd_modem_port_image_id(void) { return image_id; }
static unsigned long tick;
static int has_value, tx_owned, holding, holds, runs, sets, have_marker;
static uint8_t value;
static uint32_t marker;
static uint8_t outcome = RFD_OUT_APPLIED, flags;
unsigned long tx_time_get(void) { return tick; }
bool flight_state_cache_has_value(void) { return has_value; }
uint8_t flight_state_cache_value(void) { return value; }
uint8_t radio_uart_set_modem_hold(uint8_t on) {
  if (on && tx_owned) return 0U;
  holding = on; holds += on; return 1U;
}
int radio_uart_modem_write(const uint8_t *b, size_t n) { (void)b; (void)n; return -1; }
int radio_uart_modem_read(uint8_t *b, size_t n) { (void)b; (void)n; return -1; }
int persistent_store_get(uint32_t key, void *v, size_t *n) {
  assert(key == 0x56444652u && *n == 4U);
  if (!have_marker) return LAUNCHCORE_PERSIST_NOT_FOUND;
  memcpy(v, &marker, 4U); return LAUNCHCORE_PERSIST_OK;
}
int persistent_store_set(uint32_t key, const void *v, size_t n) {
  assert(key == 0x56444652u && n == 4U && !holding);
  memcpy(&marker, v, 4U); have_marker = 1; ++sets; return LAUNCHCORE_PERSIST_OK;
}
void __wrap_rfd_modem_run(const rfd_io_t *io, const rfd_plan_t *plan, rfd_mode_t mode,
                          const rfd_timing_t *t, rfd_gate_fn gate, void *ctx, bool save,
                          rfd_report_t *out) {
  (void)io; (void)plan; (void)t; (void)save;
  assert(holding && gate(ctx));
  ++runs; rfd_report_skipped(out, mode, 0U); out->outcome = outcome; out->flags = flags;
}
static void service(void) {}
static void poll_for(unsigned long ms) {
  for (unsigned long end = tick + ms; tick < end; tick += 5U) rfd_modem_port_poll(service);
}
static void station_write(uint8_t state) { has_value = 1; value = state; ++g_flight_state_cache_updates; }

int main(int argc, char **argv) {
  assert(argc == 2 || argc == 3);
  if (argc == 3) image_id = (uint32_t)strtoul(argv[2], NULL, 10);
  tick = 1000U;
  unsigned long boot_marker = 0;                 /* "<n>" or "p<n>": boot with this marker */
  const int marked = sscanf(argv[1] + (argv[1][0] == 'p'), "%lu", &boot_marker) == 1;
  if (marked) { have_marker = 1; marker = (uint32_t)boot_marker; }
  rfd_modem_port_poll(service);                  /* boot pass: before any write */
  if (!strcmp(argv[1], "receipt")) {
    poll_for(5000U); assert(holds == 0);           /* no FLIGHT_STATE yet */
    station_write(1); rfd_modem_port_poll(service);
    assert(holds == 0);                            /* the write's receipt is still owned */
    poll_for(RFD_RECEIPT_SETTLE_MS - 10U); assert(holds == 0);
    station_write(1);                              /* a retransmission restarts the settle */
    poll_for(RFD_RECEIPT_SETTLE_MS - 10U); assert(holds == 0);
    tx_owned = 1; poll_for(2000U); assert(holds == 0 && runs == 0);
    tx_owned = 0; poll_for(10U); assert(holds == 1 && runs == 1 && !holding);
    assert(sets == 1);                             /* applied and verified: remembered */
    station_write(0); poll_for(60000U); assert(holds == 1 && runs == 1); /* once per boot */
  } else if (!strcmp(argv[1], "states")) {
    /* PreFill, FillTest, Aborted and flight phases never hold the radio. */
    const uint8_t never[] = {2, 3, 4, 7, 9, 15};
    for (unsigned i = 0; i < sizeof never; i++) { station_write(never[i]); poll_for(30000U); }
    assert(holds == 0);
    station_write(0); poll_for(RFD_RECEIPT_SETTLE_MS + 10U); assert(holds == 1);
  } else if (!strcmp(argv[1], "marker_skip")) {
    /* Learn the image's signature, then run as a fresh boot with it. */
    station_write(1); poll_for(RFD_RECEIPT_SETTLE_MS + 10U); assert(sets == 1);
    printf("%u\n", (unsigned)marker); return 0;
  } else if (argv[1][0] == 'p') {
    /* Verified earlier, but no radio frame within the probation: forget the
     * marker (the modem may be swapped or reset) and check it this boot. */
    assert(have_marker && marker == (uint32_t)strtoul(argv[1] + 1, NULL, 10));
    poll_for(59000U); assert(sets == 0 && holds == 0);
    poll_for(1100U); assert(sets == 1 && marker == 0U && holds == 0);
    assert(((g_rfd_modem_status[0] >> 16) & 0xFFU) == RFD_FLAG_PENDING);
    station_write(1); poll_for(RFD_RECEIPT_SETTLE_MS + 10U);
    assert(holds == 1 && runs == 1 && sets == 2 && marker != 0U);
  } else if (!strcmp(argv[1], "mismatch")) {
    outcome = RFD_OUT_MISMATCH; flags = RFD_FLAG_ATTENTION;
    station_write(1); poll_for(RFD_RECEIPT_SETTLE_MS + 10U); assert(runs == 1 && sets == 0);
  } else if (!strcmp(argv[1], "attention")) {
    outcome = RFD_OUT_MATCHED; flags = RFD_FLAG_ATTENTION | RFD_FLAG_FORWARDING;
    station_write(1); poll_for(RFD_RECEIPT_SETTLE_MS + 10U); assert(runs == 1 && sets == 0);
  } else {
    assert(marked);
    /* A verified boot of the same image: no hold while the link is up. */
    poll_for(10000U); g_radio_rx_frames_ok = 1U;
    station_write(1); poll_for(120000U);
    if (holds != 0 || runs != 0 || sets != 0) return 3;
    assert(((g_rfd_modem_status[0] >> 4) & 0xFU) == RFD_OUT_SKIPPED);
    assert(((g_rfd_modem_status[0] >> 16) & 0xFFU) == RFD_FLAG_VERIFIED);
    return 0;
  }
  printf("rfd port %s passed\n", argv[1]);
  return 0;
}
'''

HOLD_HARNESS = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
static int uart; static int *g_huart = &uart;
static volatile uint8_t g_tx_dma_busy, g_radio_modem_hold, g_partial_frame_waiting;
static volatile uint32_t g_tx_count, g_frame_len;
static int cleared;
static void radio_rx_ring_clear(void) { ++cleared; }
@HOLD@
int main(void) {
  g_tx_count = 1U;               /* a queued ReliableAck */
  assert(radio_uart_set_modem_hold(1U) == 0U && !g_radio_modem_hold && !cleared);
  g_tx_count = 0U; g_tx_dma_busy = 1U;
  assert(radio_uart_set_modem_hold(1U) == 0U && !g_radio_modem_hold);
  g_tx_dma_busy = 0U;
  assert(radio_uart_set_modem_hold(1U) == 1U && g_radio_modem_hold);
  g_tx_count = 3U;               /* releasing is always allowed */
  assert(radio_uart_set_modem_hold(0U) == 1U && !g_radio_modem_hold);
  puts("hold admission passed");
  return 0;
}
'''


class RfdModemHoldGateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="rfd-hold-gate-")
        root = Path(cls.temp.name)
        for name, text in STUBS.items():
            (root / name).parent.mkdir(exist_ok=True)
            (root / name).write_text(text)
        (root / "harness.c").write_text(PORT_HARNESS)
        cls.port = root / "port"
        subprocess.run(
            ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
             "-fsanitize=address,undefined", "-DRFD_MODEM_CHECK_MODE=2",
             "-I", str(root), "-I", str(ROOT / "Core/Inc"),
             str(ROOT / "Core/Src/rfd_modem_port.c"), str(ROOT / "Core/Src/rfd_modem.c"),
             str(root / "harness.c"), "-Wl,--wrap=rfd_modem_run", "-o", str(cls.port)],
            check=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def run_port(self, case):
        return subprocess.run([str(self.port), case], check=True, capture_output=True,
                              text=True).stdout

    def test_hold_waits_for_the_write_receipt_and_owned_tx(self):
        self.run_port("receipt")

    def test_only_startup_and_idle_hold_the_radio(self):
        self.run_port("states")

    def test_unverified_outcomes_are_not_remembered(self):
        self.run_port("mismatch")
        self.run_port("attention")

    def boot(self, case, image=1):
        return subprocess.run([str(self.port), case, str(image)], capture_output=True,
                              text=True).returncode

    def test_verified_profile_skips_the_check_on_later_boots_of_the_same_image(self):
        marker = self.run_port("marker_skip").split()[0]
        self.assertEqual(self.boot(marker), 0)
        # A new firmware image, or another profile's marker, checks again.
        self.assertNotEqual(self.boot(marker, image=2), 0)
        self.assertNotEqual(self.boot(str((int(marker) + 1) % (1 << 32))), 0)

    def test_skipping_boot_without_a_radio_link_forgets_the_marker(self):
        marker = self.run_port("marker_skip").split()[0]
        self.assertEqual(self.boot("p" + marker), 0)

    def test_radio_refuses_the_hold_while_tx_owns_frames(self):
        radio = (ROOT / "Core/Src/radio.c").read_text()
        start = radio.index("uint8_t radio_uart_set_modem_hold(uint8_t on)\n{")
        end = radio.index("\n}\n", start) + 3
        with tempfile.TemporaryDirectory() as d:
            src = Path(d) / "hold.c"
            src.write_text(HOLD_HARNESS.replace("@HOLD@", radio[start:end]))
            exe = Path(d) / "hold"
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-Wno-unused-function", str(src), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True, capture_output=True)


if __name__ == "__main__":
    unittest.main()
