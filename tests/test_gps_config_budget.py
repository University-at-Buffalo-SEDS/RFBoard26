import subprocess
import tempfile
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]


class GpsConfigurationBudgetTests(unittest.TestCase):
    def test_missing_or_slow_gps_cannot_extend_startup_past_watchdog(self):
        source = (ROOT/'Core/Src/neom9n_config.c').read_text()
        def function(name):
            start=source.index('bool '+name+'(')
            end=source.index('\n}',start)+2
            return source[start:end]
        code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define UBX_CLASS_CFG 6U
#define UBX_CFG_VALSET 0x8aU
#define CFG_NAVSPG_DYNMODEL 1U
#define CFG_RATE_MEAS 2U
#define CFG_RATE_NAV 3U
#define CFG_MSGOUT_NMEA_ID_GGA_SPI 4U
#define CFG_MSGOUT_NMEA_ID_RMC_SPI 5U
#define CFG_MSGOUT_NMEA_ID_GNS_SPI 6U
#define CFG_MSGOUT_NMEA_ID_GLL_SPI 7U
#define CFG_MSGOUT_NMEA_ID_GSA_SPI 8U
#define CFG_MSGOUT_NMEA_ID_GSV_SPI 9U
#define CFG_MSGOUT_NMEA_ID_VTG_SPI 10U
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct {
 uint32_t dynamic_model, update_rate_ms;
 bool enable_gga, enable_rmc, enable_gns;
 uint8_t layers;
} gps_config_t;
static uint32_t now, started, mode, calls, ack_calls, previous_budget;
static uint32_t HAL_GetTick(void) { return now; }
static void tx_thread_sleep(uint32_t ticks) { now+=ticks; }
static bool ubx_send_message(SPI_HandleTypeDef *spi, uint8_t cls, uint8_t id,
 const uint8_t *payload, uint16_t size, uint32_t timeout) {
 (void)spi; assert(cls==6 && id==0x8a && size>=9);
 assert(timeout>0 && timeout<=previous_budget); previous_budget=timeout;
 calls++; assert(payload[1]==1 && payload[4]==calls);
 if(mode==1) { now+=timeout; return false; }
 now++; return true;
}
static bool ubx_wait_for_ack(SPI_HandleTypeDef *spi, uint8_t cls, uint8_t id, uint32_t timeout) {
 (void)spi;(void)cls;(void)id;
 assert(timeout>0 && timeout<previous_budget); ack_calls++;
 if(mode==0) { now+=timeout; return false; }
 if(mode==2) { now+=timeout-1; return true; }
 now+=5; return true;
}
''' + function('ubx_set_config') + '\n' + function('gps_configure_custom') + r'''
static void reset(uint32_t start, uint32_t test_mode) {
 now=started=start; mode=test_mode; calls=ack_calls=0; previous_budget=3000;
}
int main(void) {
 SPI_HandleTypeDef spi={0}; gps_config_t cfg={8,250,true,true,false,1};
 // Missing ACK, stuck send, and a very slow ACK all remain inside one budget.
 for(unsigned failure=0;failure<3;failure++) {
  reset(5000,failure); assert(!gps_configure_custom(&spi,&cfg,3000));
  assert(calls==1 && (uint32_t)(now-started)<=3000);
  assert(now<16384); // After the real five-second GPS boot delay.
 }
 reset(UINT32_MAX-1000U,0);
 assert(!gps_configure_custom(&spi,&cfg,3000));
 assert((uint32_t)(now-started)==3000 && calls==1);
 reset(5000,3); assert(gps_configure_custom(&spi,&cfg,3000));
 assert(calls==10 && ack_calls==10 && (uint32_t)(now-started)<3000);
 reset(0,3); assert(!gps_configure_custom(&spi,&cfg,0));
 assert(!gps_configure_custom(&spi,NULL,3000) && calls==0);
}
'''
        with tempfile.TemporaryDirectory() as temp:
            src=Path(temp)/'test.c';exe=Path(temp)/'test';src.write_text(code)
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',
                '-fsanitize=address,undefined',str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
