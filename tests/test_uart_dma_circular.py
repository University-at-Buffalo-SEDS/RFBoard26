"""Circular RX must deliver cumulative HT/TC/IDLE positions exactly once."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class CircularDmaTests(unittest.TestCase):
    def test_idle_half_complete_wrap_and_duplicate_events(self):
        harness = r'''
#include <assert.h>
#include <string.h>
#include "uart_dma_rx.h"
static unsigned used;
static uint8_t output[2048];
static void sink(const uint8_t *data,uint16_t count) {
 assert(used+count<=sizeof(output)); memcpy(output+used,data,count); used+=count;
}
static void run(uint16_t capacity) {
 uint8_t buffer[512];for(unsigned i=0;i<capacity;i++)buffer[i]=(uint8_t)i;
 uint16_t previous=0;used=0;
 uart_dma_rx_consume(buffer,capacity,0,&previous,sink);assert(used==0);
 uart_dma_rx_consume(buffer,capacity,13,&previous,sink);assert(used==13);
 uart_dma_rx_consume(buffer,capacity,13,&previous,sink);assert(used==13);
 uart_dma_rx_consume(buffer,capacity,capacity/2,&previous,sink);
 uart_dma_rx_consume(buffer,capacity,capacity/2,&previous,sink);
 uart_dma_rx_consume(buffer,capacity,capacity,&previous,sink);
 uart_dma_rx_consume(buffer,capacity,capacity,&previous,sink);assert(used==capacity);
 uart_dma_rx_consume(buffer,capacity,7,&previous,sink);assert(used==capacity+7U);
 uart_dma_rx_consume(buffer,capacity,capacity/2,&previous,sink);
 uart_dma_rx_consume(buffer,capacity,capacity,&previous,sink);assert(used==2U*capacity);
 for(unsigned i=0;i<used;i++)assert(output[i]==buffer[i%capacity]);
 uart_dma_rx_consume(buffer,capacity,capacity+1,&previous,sink);
 uart_dma_rx_consume(buffer,capacity,0,&previous,sink);assert(used==2U*capacity);
 // Restart after a hardware error starts a fresh generation at position zero.
 previous=0;uart_dma_rx_consume(buffer,capacity,17,&previous,sink);
 assert(used==2U*capacity+17U && !memcmp(output+2U*capacity,buffer,17));
}
int main(void){run(256);run(512);}
'''
        with tempfile.TemporaryDirectory() as directory:
            exe = str(Path(directory)/'circular')
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',exe],input=harness,text=True,check=True)
            subprocess.run([exe],check=True)
