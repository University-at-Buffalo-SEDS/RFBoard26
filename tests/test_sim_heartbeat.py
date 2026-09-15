"""Compile the board's actual simulation heartbeat scheduler."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SimulationHeartbeat(unittest.TestCase):
    def test_persistent_cadence_retry_clock_changes_and_release_noop(self):
        compiler = shutil.which(os.environ.get("CC", "cc"))
        if compiler is None:
            self.skipTest("host C compiler unavailable")
        header = (ROOT / "Core/Inc/sim_network_probe.h").read_text()
        function = header[header.index("static inline void sim_probe_emit_heartbeat"):]
        function = function.rsplit("#endif", 1)[0]
        prefix = """
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
typedef struct { int value; } SedsRouter;
enum { SEDS_OK = 0, SEDS_ERR = 1, SEDS_DT_HEARTBEAT = 3 };
volatile uint32_t g_sim_heartbeat_attempts, g_sim_heartbeat_ok, g_sim_heartbeat_fail;
static unsigned calls;
static int mock_status;
int seds_router_log_bytes_ex(SedsRouter *r, int type, const uint8_t *p,
                            size_t n, void *dest, int priority) {
    assert(r && type == SEDS_DT_HEARTBEAT && p && n == 0 && dest == NULL);
    assert(priority == 1);
    ++calls;
    return mock_status;
}
"""
        main = """
int main(void) {
    SedsRouter router = {0};
    for (unsigned i = 0; i < 4; ++i)
        sim_probe_emit_heartbeat(&router, 3000 + i * 100);
#ifdef SEDS_FIRMWARE_SIM_TEST
    assert(calls == 4);
    sim_probe_emit_heartbeat(&router, 3500);
    assert(calls == 4);
    uint64_t epoch = UINT64_C(1) << 40;
    sim_probe_emit_heartbeat(&router, epoch);
    sim_probe_emit_heartbeat(&router, epoch + 1);
    assert(calls == 5);
    mock_status = SEDS_ERR;
    sim_probe_emit_heartbeat(&router, epoch + 1000);
    sim_probe_emit_heartbeat(&router, epoch + 1020);
    assert(calls == 6 && g_sim_heartbeat_fail == 1);
    mock_status = SEDS_OK;
    sim_probe_emit_heartbeat(&router, epoch + 1050);
    assert(calls == 7);
    sim_probe_emit_heartbeat(&router, 500);
    assert(calls == 8);
    sim_probe_emit_heartbeat(&router, 501);
    assert(calls == 8);
#else
    assert(calls == 0 && g_sim_heartbeat_attempts == 0);
#endif
    return 0;
}
"""
        with tempfile.TemporaryDirectory(prefix="seds-heartbeat-test-") as directory:
            source = Path(directory) / "test.c"
            source.write_text(prefix + function + main)
            for enabled in (False, True):
                with self.subTest(simulation=enabled):
                    executable = Path(directory) / ("sim" if enabled else "release")
                    flags = ["-DSEDS_FIRMWARE_SIM_TEST"] if enabled else []
                    subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                                    *flags, str(source), "-o", str(executable)], check=True)
                    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    unittest.main()

