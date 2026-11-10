"""RF reports GPS status every telemetry period, with or without a module.

The station must tell "no module response" from "module alive, no fix" from
"fix" and from "no data path". The real GPS thread and NMEA driver run here
against a scripted SPI bus and a virtual clock.
"""
import json
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

GPS_STATUS_ID = 240254471  # canonical name identity of GPS_STATUS

STUBS = {
    'stm32g4xx_hal.h': r'''#pragma once
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, const uint8_t *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
static inline uint32_t __get_PSP(void) { return 0U; }
''',
    'stm32g4xx_hal_def.h': '#pragma once\n',
    'stm32g4xx_hal_spi.h': '#pragma once\n',
    'main.h': '#pragma once\n#include "stm32g4xx_hal.h"\nextern GPIO_TypeDef gps_port;\n#define GPS_CS_Pin 1U\n#define GPS_CS_GPIO_Port (&gps_port)\n',
    'board_watchdog.h': '#pragma once\n#include <stdint.h>\n#define BOARD_WATCHDOG_ACQUISITION (1UL << 3)\nvoid board_watchdog_progress(uint32_t);\n',
    'RF-Threads.h': '#pragma once\n',
    'can_bus.h': '#pragma once\n',
    'neom9n_config.h': '#pragma once\n#include <stdbool.h>\n#include "stm32g4xx_hal.h"\nbool gps_config(SPI_HandleTypeDef *, uint32_t);\n',
    'telemetry_rate.h': '#pragma once\n#include <stdint.h>\nuint32_t rf_telemetry_period_ms(void);\n',
    'tx_api.h': r'''#pragma once
#include <stdint.h>
typedef unsigned long ULONG; typedef unsigned int UINT; typedef unsigned char UCHAR; typedef void VOID;
typedef struct { int unused; } TX_BYTE_POOL;
typedef struct { void *tx_thread_stack_start, *tx_thread_stack_end, *tx_thread_stack_highest_ptr; } TX_THREAD;
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define TX_NO_TIME_SLICE 0U
#define TX_AUTO_START 1U
#define TX_SUCCESS 0U
ULONG tx_time_get(void);
UINT tx_thread_sleep(ULONG);
UINT tx_thread_create(TX_THREAD *, char *, void (*)(ULONG), ULONG, void *, ULONG, UINT, UINT, ULONG, UINT);
''',
    'tx_thread.h': '#pragma once\n#include "tx_api.h"\nvoid _tx_thread_stack_analyze(TX_THREAD *);\n',
    'telemetry.h': r'''#pragma once
#include <stddef.h>
#include <stdint.h>
#include "sedsnet_config.h"
typedef uint32_t SedsDataType;
typedef int SedsResult;
#define SEDS_OK 0
SedsResult log_telemetry_asynchronous(SedsDataType, const void *, size_t, size_t);
void telemetry_set_unix_time_ms(uint64_t);
''',
}

HARNESS = r'''
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "neom9n.h"
#include "telemetry.h"
#include "tx_api.h"

GPIO_TypeDef gps_port;
SPI_HandleTypeDef hspi1;
void neom9n_thread_entry(ULONG);

static uint32_t now_ms, end_ms;
static jmp_buf stop;
/* Bus scripts. 0: no module (0xFF idle). 1: SPI fault. 2: module streaming
 * sentences (fix or no fix). 3: streaming, then silent from silent_at_ms. */
static int mode;
static uint32_t silent_at_ms;
static const char *stream;
static size_t stream_pos;

#define MAX_LOG 512
static struct { uint32_t at; float v[4]; } status_log[MAX_LOG];
static unsigned status_count, sat_count;

uint32_t HAL_GetTick(void) { return now_ms; }
ULONG tx_time_get(void) { return now_ms; }
UINT tx_thread_sleep(ULONG ticks) {
    now_ms += (uint32_t)ticks;
    if (now_ms >= end_ms) longjmp(stop, 1);
    return 0;
}
UINT tx_thread_create(TX_THREAD *t, char *n, void (*e)(ULONG), ULONG i, void *s,
                      ULONG z, UINT p, UINT q, ULONG sl, UINT a) {
    (void)t;(void)n;(void)e;(void)i;(void)s;(void)z;(void)p;(void)q;(void)sl;(void)a; return 0;
}
void _tx_thread_stack_analyze(TX_THREAD *t) { (void)t; }
void board_watchdog_progress(uint32_t m) { (void)m; }
uint32_t rf_telemetry_period_ms(void) { return 1000U; }
void telemetry_set_unix_time_ms(uint64_t ms) { (void)ms; }
/* A faulted SPI fails the first UBX write at once; a missing module spends
 * the whole ACK budget polling idle bytes. */
bool gps_config(SPI_HandleTypeDef *h, uint32_t t) { (void)h; if (mode != 1) now_ms += t; return false; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t n, GPIO_PinState s) { (void)p;(void)n;(void)s; }
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }
void gps_offset_update_ms(int64_t o) { (void)o; }
uint64_t utc_to_epoch_ms(uint8_t d, uint8_t mo, uint8_t y, uint8_t h, uint8_t mi,
                         uint8_t s, uint16_t ms) {
    (void)d;(void)mo;(void)y;(void)h;(void)mi;(void)s;(void)ms; return 0;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, const uint8_t *tx,
                                          uint8_t *rx, uint16_t n, uint32_t timeout) {
    (void)h;(void)tx;(void)timeout;
    if (mode == 1) return HAL_ERROR;
    for (uint16_t i = 0; i < n; ++i) {
        if (mode == 0 || (mode == 3 && now_ms >= silent_at_ms)) { rx[i] = 0xFF; continue; }
        rx[i] = (uint8_t)stream[stream_pos++];
        if (stream[stream_pos] == '\0') stream_pos = 0;
    }
    return HAL_OK;
}

SedsResult log_telemetry_asynchronous(SedsDataType type, const void *data,
                                      size_t count, size_t size) {
    if (type == SEDS_DT_GPS_SATELLITE_NUMBER) ++sat_count;
    if (type != SEDS_DT_GPS_STATUS) return SEDS_OK;
    assert(count == 4 && size == sizeof(float) && status_count < MAX_LOG);
    status_log[status_count].at = now_ms;
    memcpy(status_log[status_count].v, data, sizeof(float) * 4);
    ++status_count;
    return SEDS_OK;
}

static void run(int bus, const char *text, uint32_t silent_at, uint32_t until) {
    mode = bus; stream = text; stream_pos = 0; silent_at_ms = silent_at;
    now_ms = 0; end_ms = until; status_count = 0; sat_count = 0;
    if (!setjmp(stop)) neom9n_thread_entry(0);
}

/* Status arrives at the telemetry period from soon after boot, whatever the bus does. */
static void assert_periodic(uint32_t until) {
    assert(status_count > 0);
    assert(status_log[0].at <= 10000U);
    for (unsigned i = 1; i < status_count; ++i) {
        const uint32_t gap = status_log[i].at - status_log[i - 1].at;
        assert(gap >= 900U && gap <= 1100U);
    }
    assert(until - status_log[status_count - 1].at <= 1100U);
}

#define LAST(i) (status_log[status_count - 1].v[i])
#define NO_FIX_GGA "$GNGGA,123519.00,,,,,0,03,99.99,,,,,,*5B\r\n"
#define FIX_GGA "$GNGGA,123519.00,4300.000,N,07846.800,W,1,08,0.9,300.0,M,,M,,*47\r\n"

int main(void) {
    /* No module on the bus: every byte reads idle 0xFF. */
    run(0, "", 0, 30000U);
    assert_periodic(30000U);
    assert(LAST(0) == 0.0f && LAST(1) == 0.0f && LAST(2) == -1.0f && LAST(3) == -1.0f);

    /* SPI peripheral fault. */
    run(1, "", 0, 30000U);
    assert_periodic(30000U);
    assert(LAST(0) == 0.0f && LAST(2) == -1.0f && LAST(3) == -1.0f);

    /* Module alive, no fix yet: still reports the satellites it tracks, which
     * separates an antenna/sky-view problem from a dead module. */
    run(2, NO_FIX_GGA, 0, 30000U);
    assert_periodic(30000U);
    assert(LAST(0) == 2.0f && LAST(1) == 3.0f);
    assert(LAST(2) >= 0.0f && LAST(2) <= 1.0f && LAST(3) == -1.0f);

    /* Module with a fix. */
    run(2, FIX_GGA, 0, 30000U);
    assert_periodic(30000U);
    assert(LAST(0) == 3.0f && LAST(1) == 8.0f);
    assert(LAST(2) >= 0.0f && LAST(2) <= 1.0f && LAST(3) >= 0.0f && LAST(3) <= 1.0f);

    /* Module with a fix that then goes silent: reported as silent, with ages growing. */
    run(3, FIX_GGA, 15000U, 30000U);
    assert_periodic(30000U);
    assert(LAST(0) == 1.0f && LAST(1) == 0.0f);
    assert(LAST(2) >= 13.0f && LAST(2) <= 16.0f && LAST(3) >= 13.0f && LAST(3) <= 16.0f);
    puts("gps status ok");
    return 0;
}
'''


class GpsStatusPublicationTests(unittest.TestCase):
    def test_schema_declares_gps_status_for_the_station(self):
        schema = json.loads((ROOT / 'config/sedsnet.json').read_text())
        entry = next((t for t in schema['types'] if t['name'] == 'GPS_STATUS'), None)
        self.assertIsNotNone(entry, 'config/sedsnet.json lacks GPS_STATUS')
        self.assertEqual(entry['element'], {'kind': 'Static', 'data_type': 'Float32', 'count': 4})
        self.assertEqual(entry['reliable'], 'Off')
        self.assertIn('ActuatorGroundStation', entry['endpoints'])
        # Above ordinary telemetry (0) so queue pressure does not shed it first,
        # below network control (200+).
        self.assertGreater(entry['priority'], 0)
        self.assertLess(entry['priority'], 200)
        header = (ROOT / 'Core/Inc/sedsnet_config.h').read_text()
        self.assertRegex(header, r'#define SEDS_DT_GPS_STATUS \(\(uint32_t\)%dU\)' % GPS_STATUS_ID)

    def test_status_is_periodic_with_and_without_a_module(self):
        header = (ROOT / 'Core/Inc/sedsnet_config.h').read_text()
        defines = '\n'.join(re.findall(r'#define SEDS_DT_\w+ \(\(uint32_t\)\d+U\)', header))
        if 'SEDS_DT_GPS_STATUS' not in defines:
            defines += '\n#define SEDS_DT_GPS_STATUS ((uint32_t)%dU)' % GPS_STATUS_ID
        with tempfile.TemporaryDirectory() as temp:
            inc = Path(temp) / 'inc'
            inc.mkdir()
            for name, text in STUBS.items():
                (inc / name).write_text(text)
            (inc / 'sedsnet_config.h').write_text('#pragma once\n#include <stdint.h>\n' + defines + '\n')
            shutil.copy(ROOT / 'Core/Inc/neom9n.h', inc / 'neom9n.h')
            shutil.copy(ROOT / 'Core/Inc/gps_time.h', inc / 'gps_time.h')
            harness = Path(temp) / 'harness.c'
            harness.write_text(HARNESS)
            exe = Path(temp) / 'gps_status'
            subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Wno-unused-function',
                            '-DTELEMETRY_ENABLED=1', '-I', str(inc),
                            str(harness), str(ROOT / 'Core/Src/neom9n.c'),
                            str(ROOT / 'Core/Src/neom9n_thread.c'), '-o', str(exe), '-lm'],
                           check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
