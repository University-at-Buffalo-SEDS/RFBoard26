"""Run the real foreground UART recovery code against transient HAL failures."""
import subprocess,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
class RadioRxRecoveryTests(unittest.TestCase):
 def test_retries_failed_rearms_preserves_tx_and_does_not_restart_quiet_rx(self):
  source=(ROOT/'Core/Src/radio.c').read_text()
  begin=source.index('static void radio_uart_recover_rx_if_needed(')
  end=source.index('\nHAL_StatusTypeDef radio_uart_send_bytes',begin)
  code=r'''
#include <stdint.h>
#include <assert.h>
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define HAL_UART_STATE_BUSY_RX 34
#define USART_CR3_DMAR 64U
typedef struct {unsigned CR3;} Instance;
typedef struct {unsigned State;} DMA;
#define HAL_DMA_STATE_BUSY 2
#define HAL_DMA_STATE_READY 1
typedef struct {Instance *Instance;unsigned RxState;DMA *hdmarx;} UART;
static DMA rx_dma={HAL_DMA_STATE_READY}; static unsigned dma_aborts;
static Instance instance; static UART uart={&instance,0,&rx_dma}; static UART *g_huart=&uart;
static unsigned g_tx_dma_busy,g_tx_dma_preserves_rx,g_rx_restart_pending,g_rx_recovery_waiting;
static uint32_t g_rx_recovery_due_ms,g_radio_rx_recoveries,g_rx_restart_errors;
static unsigned aborts,rearms,failures,abort_fail,new_error;static uint32_t tick;
static uint32_t radio_now_ms(void){return tick;}
static __attribute__((unused)) unsigned HAL_DMA_GetState(DMA *p){return p->State;}
static __attribute__((unused)) int HAL_DMA_Abort(DMA *p){assert(p==&rx_dma);++dma_aborts;p->State=HAL_DMA_STATE_READY;return HAL_OK;}
static int HAL_UART_AbortReceive(UART *p){assert(p==&uart);++aborts;
 if(instance.CR3&USART_CR3_DMAR)HAL_DMA_Abort(p->hdmarx);
 instance.CR3&=~USART_CR3_DMAR;return abort_fail;}
static int radio_uart_start_rx(void){++rearms;if(failures){--failures;return 1;}if(rx_dma.State==HAL_DMA_STATE_BUSY)return 1;rx_dma.State=HAL_DMA_STATE_BUSY;uart.RxState=HAL_UART_STATE_BUSY_RX;instance.CR3=USART_CR3_DMAR;if(new_error)g_rx_restart_pending=1;return 0;}
''' +source[begin:end]+r'''
int main(void){
 failures=2;
 radio_uart_recover_rx_if_needed();assert(rearms==1&&g_rx_restart_pending);
 tick=9;radio_uart_recover_rx_if_needed();assert(rearms==1);
 tick=10;radio_uart_recover_rx_if_needed();assert(rearms==2&&g_rx_restart_pending);
 tick=20;radio_uart_recover_rx_if_needed();assert(rearms==3&&!g_rx_restart_pending&&g_radio_rx_recoveries==1);
 for(tick=21;tick<10000;tick++)radio_uart_recover_rx_if_needed();assert(rearms==3);
 // Receive DMA can stop without a callback; the service loop detects this.
 instance.CR3=0;g_tx_dma_busy=1;g_tx_dma_preserves_rx=1;
 radio_uart_recover_rx_if_needed();assert(rearms==4&&g_tx_dma_busy&&g_tx_dma_preserves_rx);
 assert((instance.CR3&USART_CR3_DMAR)&&g_radio_rx_recoveries==2&&dma_aborts>0);
 // Deliberate legacy half-duplex receive pause is left alone.
 instance.CR3=0;g_tx_dma_preserves_rx=0;radio_uart_recover_rx_if_needed();assert(rearms==4);
 g_tx_dma_busy=0;tick=UINT32_MAX-4;failures=1;radio_uart_recover_rx_if_needed();assert(rearms==5);
 tick=4;radio_uart_recover_rx_if_needed();assert(rearms==5);
 tick=5;new_error=1;radio_uart_recover_rx_if_needed();assert(rearms==6&&g_rx_restart_pending);
 new_error=0;radio_uart_recover_rx_if_needed();assert(rearms==7&&!g_rx_restart_pending);
 assert(aborts==7&&g_rx_restart_errors==3);
}
'''
  with tempfile.TemporaryDirectory() as d:
   exe=str(Path(d)/'rx_recovery')
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',exe],input=code,text=True,check=True)
   subprocess.run([exe],check=True)
 def test_error_callback_defers_rearm_and_preserves_transmit(self):
  source=(ROOT/'Core/Src/radio.c').read_text();callback=source.split('void HAL_UART_ErrorCallback',1)[1].split('uint32_t board_health',1)[0]
  self.assertIn('g_rx_restart_pending = 1U',callback)
  self.assertNotIn('HAL_UART_Abort',callback)
  self.assertNotIn('radio_uart_start_rx()',callback)
  self.assertIn('radio_uart_recover_rx_if_needed();',source.split('void radio_uart_process_rx',1)[1])
