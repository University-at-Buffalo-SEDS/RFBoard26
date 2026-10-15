"""Exercise the production IRQ path with a finite three-slot hardware FIFO."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

def function(source, name):
    match = re.search(r'(?:static\s+(?:inline\s+)?(?:uint32_t|void|int)|void)\s+' + name + r'\([^;]*?\)\s*\{', source)
    assert match, name
    start = match.start()
    pos = match.end()
    depth = 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[start:pos]

class CanIrqDrainTests(unittest.TestCase):
    def test_fifo_drains_without_thread_service_and_irq_work_is_bounded(self):
        source = (ROOT/'Core/Src/can_bus.c').read_text()
        functions = '\n'.join(function(source, name) for name in [
            'can_bus_drain_rx_fifo', 'HAL_FDCAN_RxFifo0Callback',
            'HAL_FDCAN_RxFifo1Callback', 'rb_pop'])
        # Polling remains an explicit fallback, not the production default.
        block = source[source.index('#ifndef CAN_BUS_POLLING'):]
        block = block[:block.index('#endif')+len('#endif')]
        default = subprocess.run(['cc','-E','-P','-x','c','-'],
            input=block+'\nCAN_BUS_POLLING\n',text=True,capture_output=True,check=True)
        self.assertEqual(default.stdout.strip(), '0')
        harness = r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
typedef struct {unsigned instance;} FDCAN_HandleTypeDef;
typedef struct {uint32_t Identifier,DataLength;} FDCAN_RxHeaderTypeDef;
typedef struct {uint32_t std_id;uint8_t len,data[64];} can_bus_rx_frame_t;
#define rb_push rb_push_drop_oldest
#define HAL_OK 0
#define FDCAN_RX_FIFO0 0
#define FDCAN_RX_FIFO1 1
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE 2
static FDCAN_HandleTypeDef board,other;
static FDCAN_HandleTypeDef *g_hfdcan=&board;
static uint32_t g_own_std_id=0xffffffffU;
static uint32_t g_fdcan_rx_count,g_fdcan_irq_count;
static uint8_t g_rx_fifo0_pending,g_rx_fifo1_pending;
static unsigned fill[2],refill,pushed,reads,irq;
static can_bus_rx_frame_t g_rx_ring[8];
static uint16_t g_rx_head,g_rx_tail;
static uint16_t rb_next(uint16_t t){return (t+1)%8;}
static unsigned __get_PRIMASK(void){return irq;}
static void __disable_irq(void){irq=1;}
static void __set_PRIMASK(unsigned n){irq=n;}
static void __DMB(void){assert(irq);}
static size_t can_bus_dlc_to_len(uint32_t n){return n;}
static unsigned HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h,unsigned f){assert(h==g_hfdcan);return fill[f];}
static int HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h,unsigned f,FDCAN_RxHeaderTypeDef *hdr,uint8_t *data){
 assert(h==g_hfdcan && fill[f] && irq);reads++;
 hdr->Identifier=0x123;hdr->DataLength=64;memset(data,0x5a,64);
 if(!refill)fill[f]--;
 return HAL_OK;
}
static void rb_push_drop_oldest(uint32_t id,const uint8_t *data,uint8_t n){
 assert(irq && id==0x123 && n==64 && data[63]==0x5a);pushed++;
}
'''
        main = r'''
int main(void){
 irq=1;fill[0]=3;HAL_FDCAN_RxFifo0Callback(&board,1);
 assert(!fill[0] && pushed==3 && reads==3);
 // More bursts arrive without any network-thread/decoder service.
 for(unsigned i=0;i<100;i++){fill[1]=3;HAL_FDCAN_RxFifo1Callback(&board,2);assert(!fill[1]);}
 assert(pushed==303 && g_fdcan_rx_count==303);
 unsigned before=reads;fill[0]=3;refill=1;
 HAL_FDCAN_RxFifo0Callback(&board,1);assert(reads-before==3);
 before=reads;HAL_FDCAN_RxFifo0Callback(&other,1);HAL_FDCAN_RxFifo0Callback(&board,0);assert(reads==before);
 // Consumer protects the tail/slot from concurrent drop-oldest eviction,
 // and preserves an already-disabled interrupt mask on all exit paths.
 can_bus_rx_frame_t out;g_rx_ring[0].std_id=123;g_rx_head=1;g_rx_tail=0;
 irq=0;assert(rb_pop(&out) && out.std_id==123 && !irq && g_rx_tail==1);
 assert(!rb_pop(&out) && !irq);
 irq=1;assert(!rb_pop(&out) && irq);
 g_rx_head=2;assert(rb_pop(&out) && irq && g_rx_tail==2);
}
'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'test.c').write_text(harness+functions+main)
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Wno-unused-variable',
                '-fsanitize=address,undefined',str(p/'test.c'),'-o',str(p/'test')],check=True)
            subprocess.run([str(p/'test')],check=True)
