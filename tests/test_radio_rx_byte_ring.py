"""Exercise the actual RX ring with short IDLE fragments and bounded exhaustion."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class RadioRxByteRingTests(unittest.TestCase):
    def test_short_idle_fragments_use_the_existing_byte_capacity(self):
        source = (ROOT / 'Core/Src/radio.c').read_text()
        item = re.search(r'typedef struct \{[^}]*\} radio_rx_item_t;', source).group()
        ring = re.search(r'static (?:uint8_t|radio_rx_item_t) g_rx_ring\[[^;]+;', source).group()
        metadata = source.split('/* ISR -> thread ring buffer */',1)[1].split('static uint8_t g_rx_ring',1)[0]
        functions = source[source.index('static inline void radio_rx_ring_push_isr('):source.index('/**\n * Call this periodically', source.index('static inline void radio_rx_ring_push_isr('))]
        harness = r'''
#include <stdint.h>
#include <string.h>
#include <assert.h>
#define RADIO_UART_RX_BUF_SIZE 256U
#define RADIO_UART_RX_RING_DEPTH 4U
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
static uint32_t g_rx_isr_drops, g_rx_isr_chunks, g_rx_isr_bytes;
'''
        test = r'''
int main(void) {
 uint8_t input[256]; for (unsigned i=0;i<sizeof(input);i++) input[i]=(uint8_t)i;
 // Twenty IDLE events occupy 260 bytes, not twenty full DMA-sized slots.
 for (unsigned i=0;i<20;i++) radio_rx_ring_push_isr(input,13);
 assert(g_rx_isr_drops==0 && g_rx_isr_bytes==260);
 radio_rx_item_t out; unsigned read=0;
 while (radio_rx_ring_pop_thread(&out)) {
  assert(out.len>0 && out.len<=256);
  for(unsigned i=0;i<out.len;i++,read++) assert(out.data[i]==input[read%13]);
 }
 assert(read==260);
 // Wrap head/tail repeatedly without changing byte order or capacity.
 for (unsigned round=0;round<20;round++) {
  for(unsigned i=0;i<RADIO_UART_RX_RING_DEPTH;i++) radio_rx_ring_push_isr(input,256);
  unsigned dropped=g_rx_isr_drops;
  radio_rx_ring_push_isr(input,13); assert(g_rx_isr_drops==dropped+1);
  for(unsigned i=0;i<RADIO_UART_RX_RING_DEPTH;i++) {
   assert(radio_rx_ring_pop_thread(&out)); assert(out.len==256);
   assert(memcmp(out.data,input,256)==0);
  }
  assert(!radio_rx_ring_pop_thread(&out));
 }
 radio_rx_ring_push_isr(input,0); assert(!radio_rx_ring_pop_thread(&out));
}
'''
        with tempfile.TemporaryDirectory() as directory:
            executable = str(Path(directory) / 'rx-ring')
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',executable], input=harness+metadata+item+'\n'+ring+'\n'+functions+test,text=True,check=True)
            subprocess.run([executable],check=True)
            # A larger configured ring widens the metadata rather than reducing
            # its supported capacity or wrapping the full byte count at 65536.
            large=harness.replace('RADIO_UART_RX_RING_DEPTH 4U','RADIO_UART_RX_RING_DEPTH 256U')
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',executable], input=large+metadata+item+'\n'+ring+'\n'+functions+test,text=True,check=True)
            subprocess.run([executable],check=True)
