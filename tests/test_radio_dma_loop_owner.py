"""Compile actual RF DMA service under larger service-loop overrides."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
import test_radio_funded_packet_boundary as funded
ROOT=Path(__file__).resolve().parents[1]

def compile_case(source_root, frames, expect_old_failure=False, lora=False):
    source=(source_root/'Core/Src/radio.c').read_text()
    if expect_old_failure:
        guard='while (sent < RADIO_UART_TX_FRAMES_PER_SERVICE && !g_tx_dma_busy &&'
        assert source.count(guard)==1
        source=source.replace(guard,'while (sent < RADIO_UART_TX_FRAMES_PER_SERVICE &&')
    code='' if lora else funded.function(source,'static uint8_t radio_read_uleb128(')
    if not lora: code+=funded.function(source[source.index('/* Only canonical packets are independent'):],'static uint8_t radio_uart_independent_control(')
    code+=source[source.index('static uint32_t radio_uart_air_ms(uint16_t len)\n{'):source.index('uint8_t radio_uart_tx_ready(void)')]
    code+=funded.function(source,'uint32_t radio_uart_process_tx_with_budget(uint32_t budget_ms)\n{')
    code+=funded.function(source,'void HAL_UART_TxCpltCallback(')
    code+=funded.function(source,'void HAL_UART_ErrorCallback(')
    pre=funded.PREAMBLE.replace('#define RADIO_UART_TX_FRAMES_PER_SERVICE 1U',f'#define RADIO_UART_TX_FRAMES_PER_SERVICE {frames}U')
    start=pre.index('static unsigned radio_uart_dequeue_frame_with_budget(')
    end=pre.index('static void HAL_UART_AbortTransmit(',start)
    pre=pre[:start]+r'''
static radio_tx_item_t suffix;
static uint8_t *dma_owned;
static unsigned dma_live;
static unsigned radio_uart_dequeue_frame_with_budget(radio_tx_item_t *out,unsigned budget){
 (void)budget;if(!offered)return 0;
 *out=candidate;candidate=suffix;--offered;return 1;
}
'''+pre[end:]
    old='static unsigned HAL_UART_Transmit_DMA(void *uart,uint8_t *data,unsigned n){(void)uart;(void)data;(void)n;++sends;return hal_status;}'
    new='static unsigned HAL_UART_Transmit_DMA(void *uart,uint8_t *data,unsigned n){(void)uart;(void)n;++sends;if(dma_live)return HAL_BUSY;dma_live=1;dma_owned=data;return hal_status;}'
    assert old in pre;pre=pre.replace(old,new)
    if lora:
        pre=pre.replace('#define RFBOARD_LORA_SCHEDULER 0','#define RFBOARD_LORA_SCHEDULER 1\n#define RADIO_TX_COOLDOWN_MS 25U')
        pre=re.sub(r'static uint8_t radio_uart_ack_contracts[^;]+;','',pre)

    fixtures='\n'.join(re.search(r'static const uint8_t '+name+r'\[\]=\{[^;]+;',funded.ACK_FIXTURES).group()for name in ['und','fc'])
    checks=r'''
int main(void){
 clock_ms=1000;hal_status=HAL_OK;
 candidate.len=sizeof(und);candidate.priority=255;candidate.uart_frames=1;memcpy(candidate.data,und,sizeof(und));
 suffix.len=sizeof(fc);suffix.priority=255;suffix.uart_frames=1;memcpy(suffix.data,fc,sizeof(fc));offered=2;
 assert(radio_uart_process_tx_with_budget(2000)==1);
 assert(g_tx_dma_busy==1 && dma_live==1 && sends==1 && offered==1);
 assert(memcmp(dma_owned,und,sizeof(und))==0 && g_tx_dma_item.len==sizeof(und));
 clock_ms=1001;assert(radio_uart_process_tx_with_budget(2000)==0);
 assert(g_tx_dma_busy==1 && sends==1 && offered==1 && requeues==0);
 assert(memcmp(dma_owned,und,sizeof(und))==0);
 clock_ms=1005;dma_live=0;HAL_UART_TxCpltCallback(g_huart);
 assert(g_tx_dma_busy==0 && offered==1);
 clock_ms=1060;assert(radio_uart_process_tx_with_budget(2000)==1);
 assert(g_tx_dma_busy==1 && sends==2 && offered==0);
 assert(memcmp(dma_owned,fc,sizeof(fc))==0 && g_tx_dma_item.len==sizeof(fc));
 clock_ms=1065;dma_live=0;HAL_UART_TxCpltCallback(g_huart);
 assert(g_tx_dma_busy==0 && requeues==0 && g_tx_drops==0);
 return 0;
}
'''
    if lora:
        names=set(re.findall(r'\bg_\w+',pre))
        checks=checks.replace('int main(void){','int main(void){'+' '.join('(void)'+name+';'for name in sorted(names)))
    crc='' if lora else funded.function((source_root/'tests/radio_crc_reference.h').read_text(),'uint32_t launchcore_crc32(')
    limit=next(line for line in source.splitlines()if line.startswith('#define RADIO_PACKET_SERVICE_CREDIT_CAP_MS '))
    with tempfile.TemporaryDirectory()as tmp:
        exe=str(Path(tmp)/'dma-loop')
        subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',exe],input=pre+limit+'\n'+crc+code+fixtures+checks,text=True,check=True)
        result=subprocess.run([exe],capture_output=True,text=True)
        if expect_old_failure:
            assert result.returncode!=0 and 'g_tx_dma_busy==1' in result.stderr, result.stderr
        else:
            assert result.returncode==0,result.stderr

class DmaLoopOwnerTests(unittest.TestCase):
    def test_default_one_retains_owner_until_actual_completion(self):compile_case(ROOT,1)
    def test_override_two_retains_owner_until_actual_completion(self):compile_case(ROOT,2)
    def test_override_four_retains_owner_until_actual_completion(self):compile_case(ROOT,4)
    def test_legacy_lora_override_two_retains_owner_until_actual_completion(self):compile_case(ROOT,2,lora=True)
    def test_original_override_two_overwrites_dma_owner_negative_control(self):
        compile_case(ROOT,2,True)
