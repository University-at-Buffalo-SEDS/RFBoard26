"""Instrumented simulation must arm the same ReceiveToIdle DMA as hardware."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class RadioDmaParityTests(unittest.TestCase):
    def test_real_dma_start_and_hal_backpressure_in_both_builds(self):
        source = (ROOT / 'Core/Src/radio.c').read_text()
        start = source.index('HAL_StatusTypeDef radio_uart_start_rx(void) {')
        end = source.index('\nHAL_StatusTypeDef radio_uart_send_bytes', start)
        code = r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#define HAL_OK 0
#define HAL_BUSY 1
#define HAL_ERROR 2
#define DMA_IT_HT 1
#define __HAL_DMA_DISABLE_IT(d,f) ((void)(d),(void)(f),++disabled)
typedef int HAL_StatusTypeDef;
typedef struct {void *hdmarx;} UART_HandleTypeDef;
static UART_HandleTypeDef *g_huart;
static uint8_t g_rx_buf[512];
static unsigned calls,disabled;
static int result;
static int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,unsigned n){assert(u==g_huart && p==g_rx_buf && n==512);calls++;return result;}
''' + source[start:end] + r'''
int main(void){
 assert(radio_uart_start_rx()==HAL_ERROR && calls==0);
 UART_HandleTypeDef u={(void*)1};g_huart=&u;
 assert(radio_uart_start_rx()==HAL_OK && calls==1 && disabled==1);
 result=HAL_BUSY;assert(radio_uart_start_rx()==HAL_BUSY && calls==2 && disabled==1);
 result=HAL_ERROR;assert(radio_uart_start_rx()==HAL_ERROR && calls==3 && disabled==1);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)
            (p / 'test.c').write_text(code)
            for flags in [[], ['-DSEDS_FIRMWARE_SIM_TEST=1']]:
                subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', *flags,
                                str(p / 'test.c'), '-o', str(p / 'test')], check=True)
                subprocess.run([str(p / 'test')], check=True)

if __name__ == '__main__':
    unittest.main()
