"""Fault injection against the actual foreground recovery function, not a copied implementation."""
from pathlib import Path
import os, shutil, subprocess, tempfile, unittest

ROOT = Path(__file__).resolve().parents[1]


def recovery_sources(source):
    start = source.index('#define CAN_BUS_RECOVERY_RETRY_MS')
    end = source.index('static inline void can_bus_notify_rx(', start)
    recovery = source[start:end]
    start = source.index('static HAL_StatusTypeDef can_bus_config_accept_all(FDCAN_HandleTypeDef *hfdcan)\n{')
    end = source.index('#if !CAN_BUS_POLLING\n#define CAN_BUS_NOTIFICATIONS', start)
    filters = source[start:end]
    start = end
    end = source.index('\n#endif\n', start) + len('\n#endif\n')
    notifications = source[start:end]
    return recovery, filters, notifications


HARNESS = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
typedef enum{HAL_OK,HAL_ERROR,HAL_BUSY}HAL_StatusTypeDef;
#define HAL_FDCAN_STATE_READY 1
#define HAL_FDCAN_STATE_BUSY 2
#define HAL_FDCAN_STATE_ERROR 3
#define FDCAN_TX_BUFFER0 1
#define FDCAN_TX_BUFFER1 2
#define FDCAN_TX_BUFFER2 4
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE 2U
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST 4U
#define FDCAN_IT_RX_FIFO1_MESSAGE_LOST 8U
#define FDCAN_IT_TX_FIFO_EMPTY 16U
#define FDCAN_IT_TX_COMPLETE 32U
#define FDCAN_IT_TX_EVT_FIFO_NEW_DATA 64U
#define BOARD_NOTIFICATIONS 127U
#define FDCAN_IT_GROUP_RX_FIFO0 1U
#define FDCAN_IT_GROUP_RX_FIFO1 2U
#define FDCAN_IT_GROUP_SMSG 4U
#define FDCAN_IT_GROUP_TX_FIFO_ERROR 8U
#define FDCAN_INTERRUPT_LINE0 1U
#define FDCAN_ACCEPT_IN_RX_FIFO0 0U
#define FDCAN_REJECT_REMOTE 1U
#define FDCAN_STANDARD_ID 0U
#define FDCAN_FILTER_MASK 2U
#define FDCAN_FILTER_TO_RXFIFO0 1U
#define FDCAN_FLAG_BUS_OFF 0x02000000U
#define CAN_BUS_POLLING 0
typedef struct{int State;uint32_t IR;}FDCAN_HandleTypeDef;
typedef struct{unsigned BusOff;}FDCAN_ProtocolStatusTypeDef;
typedef struct{uint32_t IdType,FilterIndex,FilterType,FilterConfig,FilterID1,FilterID2;}FDCAN_FilterTypeDef;
#define __HAL_FDCAN_GET_FLAG(h,f) ((h)->IR & (f))
#define __HAL_FDCAN_CLEAR_FLAG(h,f) ((h)->IR &= ~(f))
static FDCAN_HandleTypeDef handle={HAL_FDCAN_STATE_BUSY,0},*g_hfdcan=&handle;
static unsigned bus_off,stop_timeout,fail_init,fail_global,fail_filter,fail_lines,fail_notify,starts,stops,inits,resets,notifications,lines,filters;
static uint32_t now,g_notification_mask;
static uint8_t g_can_recovering;
volatile uint32_t g_can_tx_service_stage;
uint32_t HAL_GetTick(void){return now;}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef*h,FDCAN_ProtocolStatusTypeDef*p){(void)h;p->BusOff=bus_off;return HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef*h,unsigned n){assert(g_can_recovering);assert(n==7);(void)h;return HAL_OK;}
/* Checked-in ST HAL state contract: Stop timeout leaves ERROR. Start needs READY. */
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef*h){assert(g_can_recovering);++stops;if(h->State!=HAL_FDCAN_STATE_BUSY)return HAL_ERROR;if(stop_timeout){stop_timeout=0;h->State=HAL_FDCAN_STATE_ERROR;return HAL_ERROR;}h->State=HAL_FDCAN_STATE_READY;return HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef*h){assert(g_can_recovering);++inits;if(fail_init)return HAL_ERROR;h->State=HAL_FDCAN_STATE_READY;return HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef*h){assert(g_can_recovering);++starts;if(h->State!=HAL_FDCAN_STATE_READY)return HAL_ERROR;h->State=HAL_FDCAN_STATE_BUSY;bus_off=0;return HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef*h,uint32_t a,uint32_t b,uint32_t c,uint32_t d){assert(g_can_recovering);assert(h->State==HAL_FDCAN_STATE_READY);assert(a==FDCAN_ACCEPT_IN_RX_FIFO0&&b==FDCAN_ACCEPT_IN_RX_FIFO0&&c==FDCAN_REJECT_REMOTE&&d==FDCAN_REJECT_REMOTE);return fail_global?HAL_ERROR:HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef*h,FDCAN_FilterTypeDef*f){assert(g_can_recovering);assert(h->State==HAL_FDCAN_STATE_READY);assert(f->FilterIndex==0&&f->FilterType==FDCAN_FILTER_MASK&&f->FilterConfig==FDCAN_FILTER_TO_RXFIFO0&&f->FilterID2==0);++filters;return fail_filter?HAL_ERROR:HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_ConfigInterruptLines(FDCAN_HandleTypeDef*h,unsigned groups,unsigned line){assert(g_can_recovering);assert(h->State==HAL_FDCAN_STATE_BUSY);assert(groups==15&&line==FDCAN_INTERRUPT_LINE0);++lines;return fail_lines?HAL_ERROR:HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef*h,unsigned mask,unsigned buffers){assert(g_can_recovering);assert(h->State==HAL_FDCAN_STATE_BUSY);assert(mask==BOARD_NOTIFICATIONS&&buffers==7);++notifications;return fail_notify?HAL_ERROR:HAL_OK;}
void can_tx_queue_reset(void){assert(g_can_recovering);++resets;}
'''

MAIN = r'''
static void fresh(void){handle.State=HAL_FDCAN_STATE_BUSY;handle.IR=0;bus_off=stop_timeout=fail_init=fail_global=fail_filter=fail_lines=fail_notify=starts=stops=inits=resets=notifications=lines=filters=0;g_can_recovering=0;g_notification_mask=0;now=0;}
int main(void){
 /* healthy fastpath untouched, and a latched BUS_OFF interrupt flag is still cleared */
 fresh();handle.IR=FDCAN_FLAG_BUS_OFF;for(unsigned n=0;n<10000;n++){now=n;assert(can_bus_recover_if_bus_off()==HAL_OK);}assert(!starts&&!stops&&!inits&&!notifications&&!handle.IR);
 /* Stop ERROR, then BusOff clears by itself: still retried after 100 ms, never earlier */
 fresh();bus_off=1;stop_timeout=1;assert(can_bus_recover_if_bus_off()==HAL_ERROR);assert(handle.State==HAL_FDCAN_STATE_ERROR&&g_can_recovering);bus_off=0;
 for(now=1;now<100;now++){assert(can_bus_recover_if_bus_off()==HAL_BUSY);}
 assert(!starts&&!inits);
 handle.IR=FDCAN_FLAG_BUS_OFF;assert(can_bus_recover_if_bus_off()==HAL_OK);
 assert(handle.State==HAL_FDCAN_STATE_BUSY&&inits==1&&starts==1&&filters==1&&lines==1&&notifications==1&&!g_can_recovering&&!handle.IR);
 assert(g_notification_mask==BOARD_NOTIFICATIONS&&g_fdcan_recovery_count==1&&g_fdcan_recovery_fail_count==1&&g_fdcan_bus_off_count==1&&g_fdcan_recovery_attempt_count==2);
 /* each later stage can fail repeatedly; admission stays gated and each retry restarts from the right state */
 fresh();handle.State=HAL_FDCAN_STATE_ERROR;fail_init=1;assert(can_bus_recover_if_bus_off()==HAL_ERROR);
 now=100;fail_init=0;fail_global=1;assert(can_bus_recover_if_bus_off()==HAL_ERROR);assert(g_can_recovering&&starts==0);
 now=200;fail_global=0;fail_filter=1;assert(can_bus_recover_if_bus_off()==HAL_ERROR);assert(g_can_recovering&&starts==0);
 now=300;fail_filter=0;fail_lines=1;assert(can_bus_recover_if_bus_off()==HAL_ERROR);assert(handle.State==HAL_FDCAN_STATE_BUSY&&g_can_recovering);
 now=400;fail_lines=0;fail_notify=1;assert(can_bus_recover_if_bus_off()==HAL_ERROR);assert(handle.State==HAL_FDCAN_STATE_BUSY&&g_can_recovering);
 now=500;fail_notify=0;assert(can_bus_recover_if_bus_off()==HAL_OK);assert(!g_can_recovering&&notifications==3&&stops==2&&starts==3&&resets==3);
 /* a controller left READY (boot Start failure) is started without Stop or Init */
 fresh();handle.State=HAL_FDCAN_STATE_READY;assert(can_bus_recover_if_bus_off()==HAL_OK);assert(starts==1&&notifications==1&&inits==0&&stops==0);
 /* retry interval survives HAL tick wrap */
 fresh();now=UINT32_MAX-49;bus_off=1;stop_timeout=1;assert(can_bus_recover_if_bus_off()==HAL_ERROR);bus_off=0;now=49;assert(can_bus_recover_if_bus_off()==HAL_BUSY);now=50;assert(can_bus_recover_if_bus_off()==HAL_OK);assert(starts==1);
 /* A second real bus fault is independently recoverable, with original queue reset. */
 now=150;bus_off=1;assert(can_bus_recover_if_bus_off()==HAL_OK);assert(starts==2&&resets==2);
 puts("healthy10000passes; StopERROR/clearedBO; repeatedinit/filter/IRQfaults; READY; wrap; repeatedbusoff: PASS");return 0;}
'''


class CanRestartRecovery(unittest.TestCase):
    def test_fault_clear_restarts_and_notification_failure_retries_without_resetting_sdk(self):
        source = Path(os.environ.get('CAN_RECOVERY_SOURCE', str(ROOT / 'Core/Src/can_bus.c'))).read_text()
        recovery, filters, notifications = recovery_sources(source)
        code = HARNESS + recovery + filters + notifications + MAIN
        with tempfile.TemporaryDirectory() as directory:
            c = Path(directory) / 'recovery.c'
            exe = Path(directory) / 'recovery'
            c.write_text(code)
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                            '-fno-omit-frame-pointer', str(c), '-o', str(exe)], check=True)
            evidence = os.environ.get('CAN_RECOVERY_EVIDENCE')
            if evidence:
                output = Path(evidence)
                output.mkdir(parents=True, exist_ok=False)
                shutil.copy2(c, output / c.name)
                shutil.copy2(exe, output / exe.name)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
