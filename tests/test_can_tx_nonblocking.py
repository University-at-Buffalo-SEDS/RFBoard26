"""A full controller must not block the acquisition/network worker."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class CanTxNonblockingTests(unittest.TestCase):
    def test_production_enqueue_refuses_full_hardware_without_waiting(self):
        source = (ROOT / 'Core/Src/can_bus.c').read_text()
        start = source.index('HAL_StatusTypeDef can_bus_send_bytes(')
        end = source.index('HAL_StatusTypeDef can_bus_send_large(', start)
        function = source[start:end]
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef struct { uint32_t Identifier,IdType,TxFrameType,DataLength,
 ErrorStateIndicator,BitRateSwitch,FDFormat,TxEventFifoControl,MessageMarker; } FDCAN_TxHeaderTypeDef;
#define FDCAN_STANDARD_ID 0
#define FDCAN_DATA_FRAME 0
#define FDCAN_ESI_ACTIVE 0
#define FDCAN_BRS_OFF 0
#define FDCAN_CLASSIC_CAN 0
#define FDCAN_FD_CAN 1
#define FDCAN_NO_TX_EVENTS 0
static void *g_hfdcan=(void*)1;
static unsigned g_can_recovering,free_slots,reads,adds;
static uint32_t g_fdcan_tx_ok_count,g_fdcan_tx_fail_count;
static HAL_StatusTypeDef result=HAL_OK;
static unsigned HAL_FDCAN_GetTxFifoFreeLevel(void *h){assert(h==g_hfdcan);reads++;return free_slots;}
static size_t can_bus_round_up_fd_len(size_t n){return n;}
static uint32_t can_bus_len_to_dlc(size_t n){return (uint32_t)n;}
static HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(void *h,const FDCAN_TxHeaderTypeDef *hdr,const uint8_t *p){
 assert(h==g_hfdcan && hdr->Identifier==0x101 && hdr->DataLength==64 && p[63]==63);adds++;return result;
}
''' + function + r'''
int main(void){
 uint8_t data[128];for(unsigned i=0;i<128;i++)data[i]=(uint8_t)i;
 assert(can_bus_send_bytes(data,128,0x101)==HAL_BUSY);
 assert(reads==1 && !adds && !g_fdcan_tx_fail_count);
 // Repeated backpressure remains one register check per call, no time loop.
 for(unsigned n=0;n<100;n++)assert(can_bus_send_bytes(data,128,0x101)==HAL_BUSY);
 assert(reads==101 && !adds);
 free_slots=1;assert(can_bus_send_bytes(data,128,0x101)==HAL_OK);
 assert(adds==1 && g_fdcan_tx_ok_count==1);
 result=HAL_ERROR;assert(can_bus_send_bytes(data,128,0x101)==HAL_ERROR);
 assert(adds==2 && g_fdcan_tx_fail_count==1);
 g_can_recovering=1;assert(can_bus_send_bytes(data,128,0x101)==HAL_ERROR && adds==2);
 g_can_recovering=0;assert(can_bus_send_bytes(NULL,1,0)==HAL_ERROR);
 assert(can_bus_send_bytes(data,0,0)==HAL_ERROR);
 g_hfdcan=NULL;assert(can_bus_send_bytes(data,128,0)==HAL_ERROR);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c=Path(tmp)/'test.c';exe=Path(tmp)/'test';c.write_text(code)
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(c),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
