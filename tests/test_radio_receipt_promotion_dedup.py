"""Recent logical receipts retain retry bytes without repeated modem closes."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_radio_funded_packet_boundary import PREAMBLE, function
ROOT = Path(__file__).resolve().parents[1]
ACK_AND_NORMAL_FIXTURES = r'''
static const uint8_t d0[]={165,90,63,0,83,68,84,1,149,252,188,138,11,136,1,101,24,231,211,234,182,183,51,163,133,1,150,253,232,225,3,249,140,173,91,0,0,0,0,0,0,0,0,229,59,236,65,0,0,0,0,0,0,0,0,0,0,0,0,209,129,37,31,131,123,74,246};
static const uint8_t d1[]={165,90,51,0,12,1,1,8,224,218,234,182,183,51,223,164,2,225,206,200,143,10,20,3,68,2,2,40,235,83,222,98,111,66,65,95,67,146,158,18,187,252,167,168,4,177,163,154,67,18,150,216,176,94,13};
static const uint8_t d2[]={165,90,46,0,83,68,84,1,229,142,222,81,136,3,106,0,131,226,234,182,183,51,245,222,3,219,252,172,242,14,248,181,196,85,249,140,173,91,132,154,150,127,91,243,145,8,245,71,51,157};
static const uint8_t d3[]={165,90,66,0,206,2,173,175,182,24,28,0,181,53,150,253,232,225,3,100,249,140,173,91,11,7,13,1,95,67,146,158,18,187,252,167,132,0,87,65,82,78,58,32,114,101,115,101,114,118,101,32,104,101,97,112,32,116,111,111,32,115,109,97,108,108,201,220,227,242};
static const uint8_t d4[]={165,90,46,0,83,68,84,1,206,217,193,253,6,136,3,106,0,223,210,234,182,183,51,247,178,1,238,218,132,102,248,181,196,85,249,140,173,91,132,154,150,127,75,211,107,49,234,14,27,48};
static const uint8_t d5[]={165,90,63,0,83,68,84,1,149,252,188,138,11,136,1,101,24,221,216,234,182,183,51,193,192,3,150,253,232,225,3,249,140,173,91,0,0,0,0,0,0,0,0,229,59,236,65,0,0,0,0,0,0,0,0,0,0,0,0,236,51,203,69,131,123,74,246};
static const uint8_t d6[]={165,90,51,0,12,1,1,8,179,221,234,182,183,51,237,235,3,225,206,200,143,10,20,3,68,2,2,40,235,83,222,98,111,66,65,95,67,146,158,18,187,252,167,168,4,177,163,154,67,18,150,53,108,119,214};
static const uint8_t d7[]={165,90,50,0,12,1,1,8,253,235,234,182,183,51,153,89,164,134,212,212,6,20,3,68,2,2,100,66,44,90,241,216,214,133,95,67,146,158,18,187,252,167,28,66,98,81,243,4,33,107,199,0,247,10};
static const uint8_t d8[]={165,90,51,0,12,1,1,8,133,236,234,182,183,51,255,212,3,164,134,212,212,6,20,3,68,2,2,100,66,44,90,241,216,214,133,95,67,146,158,18,187,252,167,186,224,22,62,240,32,147,166,179,33,153,36};
static const uint8_t d9[]={165,90,51,0,12,1,1,8,167,236,234,182,183,51,217,247,3,164,134,212,212,6,20,3,68,2,2,100,66,44,90,241,216,214,133,95,67,146,158,18,187,252,167,39,215,204,208,250,201,155,11,200,98,100,124};
static const uint8_t baro[]={165,90,36,0,83,68,84,2,136,224,226,162,135,3,12,187,180,236,182,183,51,229,169,2,0,0,0,0,0,80,195,71,0,248,64,194,2,6,52,227};

'''
RECOVERY_CHECKS = r'''

static void reset_all(void){g_tx_packet_active=g_tx_air_budget_active=g_tx_dma_busy=g_tx_dma_preserves_rx=0;g_tx_last_forced_boundary_ms=g_tx_completed_service_credit_ms=g_tx_pending_service_ms=0;g_tx_closed_ack_count=g_tx_closed_ack_cursor=0;memset(g_tx_closed_ack_ids,0,sizeof(g_tx_closed_ack_ids));g_tx_independent_streak=0;g_radio_tx_ok=g_tx_dma_complete=g_tx_dma_started=0;offered=0;hal_status=HAL_OK;}
static void offer(const uint8_t*b,unsigned n,unsigned p){memset(&candidate,0,sizeof(candidate));candidate.len=n;candidate.priority=p;memcpy(candidate.data,b,n);offered=1;}
static void begin(unsigned at){clock_ms=at;assert(radio_uart_process_tx_with_budget(2000)==1);}
static void done(unsigned at){clock_ms=at;HAL_UART_TxCpltCallback(g_huart);}
static void fix_crc(void){unsigned n=candidate.len;unsigned c=launchcore_crc32(candidate.data+4,n-8);for(unsigned i=0;i<4;++i)candidate.data[n-4+i]=(c>>(8*i))&255;}
int main(void){reset_all();
 offer(d0,sizeof(d0),0);begin(3385);fprintf(stderr,"DEDUP d0 IMU_DATA ack=None forced=%u credit=%u\n",g_tx_last_forced_boundary_ms,g_tx_completed_service_credit_ms);done(3391);
 offer(d1,sizeof(d1),255);begin(3411);fprintf(stderr,"DEDUP d1 ReliableAck ack=10813779986720294056 forced=%u credit=%u\n",g_tx_last_forced_boundary_ms,g_tx_completed_service_credit_ms);done(3416);
 offer(d2,sizeof(d2),253);begin(3474);fprintf(stderr,"DEDUP d2 HEARTBEAT ack=None forced=%u credit=%u\n",g_tx_last_forced_boundary_ms,g_tx_completed_service_credit_ms);done(3478);
 offer(d3,sizeof(d3),0);begin(3496);fprintf(stderr,"DEDUP d3 ORDERED_MESSAGE ack=None forced=%u credit=%u\n",g_tx_last_forced_boundary_ms,g_tx_completed_service_credit_ms);done(3502);
 offer(d4,sizeof(d4),253);begin(3646);fprintf(stderr,"DEDUP d4 HEARTBEAT ack=None forced=%u credit=%u\n",g_tx_last_forced_boundary_ms,g_tx_completed_service_credit_ms);done(3650);
 offer(d5,sizeof(d5),0);begin(3671);fprintf(stderr,"DEDUP d5 IMU_DATA ack=None forced=%u credit=%u\n",g_tx_last_forced_boundary_ms,g_tx_completed_service_credit_ms);done(3676);
 offer(d6,sizeof(d6),255);begin(3772);fprintf(stderr,"DEDUP d6 ReliableAck ack=10813779986720294056 forced=%u credit=%u\n",g_tx_last_forced_boundary_ms,g_tx_completed_service_credit_ms);done(3777);
 offer(d7,sizeof(d7),255);begin(3833);fprintf(stderr,"DEDUP d7 ReliableAck ack=7719456679378698780 forced=%u credit=%u\n",g_tx_last_forced_boundary_ms,g_tx_completed_service_credit_ms);done(3838);
 offer(d8,sizeof(d8),255);begin(3894);fprintf(stderr,"DEDUP d8 ReliableAck ack=12002973648078430394 forced=%u credit=%u\n",g_tx_last_forced_boundary_ms,g_tx_completed_service_credit_ms);done(3899);
 offer(d9,sizeof(d9),255);begin(3955);fprintf(stderr,"DEDUP d9 ReliableAck ack=836484234889451303 forced=%u credit=%u\n",g_tx_last_forced_boundary_ms,g_tx_completed_service_credit_ms);done(3960);
assert(g_tx_last_forced_boundary_ms==3955);

/* Lose first ACK in the receiver only. Retry still reaches physical DMA;
 * suppressing a second promotion cannot suppress bytes or bounded batching. */
reset_all();offer(d1,sizeof(d1),255);begin(1000);done(1005);assert(g_tx_closed_ack_count==1);
offer(d1,sizeof(d1),255);begin(1260);assert(g_tx_last_forced_boundary_ms==1000&&g_tx_completed_service_credit_ms==0);assert(g_tx_dma_item.len==sizeof(d1)&&memcmp(g_tx_dma_item.data,d1,sizeof(d1))==0);done(1265);
unsigned end=0,last_done=1265,finish=0,normal_count=0;
for(unsigned now=1283;now<=1450;++now){
 if(g_tx_dma_busy&&now>=finish){done(now);last_done=now;}
 clock_ms=now;if(g_tx_dma_busy||radio_uart_air_busy())continue;
 if(now>1360&&now-last_done>=55){end=now;break;}
 offer(baro,sizeof(baro),0);if(radio_uart_process_tx_with_budget(2000)){finish=now+(sizeof(baro)*10000U+115199)/115200;++normal_count;}
}
assert(end&&end<=1260+100+5+55);assert(normal_count>0&&g_tx_dma_started>=3);fprintf(stderr,"LOST_FIRST retry1260 lastnormalstop%u recovered_idle%u normalframes%u\n",last_done,end,normal_count);
/* A different owner still keeps its actual ACK bytes; packetization cache
 * is per logical contract, and no owner receipt is dropped by this guard. */
reset_all();offer(d1,sizeof(d1),255);begin(2000);done(2005);offer(d1,sizeof(d1),255);size_t off=6;uint64_t ignore;radio_read_uleb128(candidate.data,candidate.len,&off,&ignore);radio_read_uleb128(candidate.data,candidate.len,&off,&ignore);radio_read_uleb128(candidate.data,candidate.len,&off,&ignore);if(candidate.data[4]&8)radio_read_uleb128(candidate.data,candidate.len,&off,&ignore);candidate.data[off]^=1;fix_crc();uint8_t changed[1028];memcpy(changed,candidate.data,candidate.len);begin(2260);assert(g_tx_last_forced_boundary_ms==2000&&memcmp(g_tx_dma_item.data,changed,candidate.len)==0);done(2265);
/* Four remembered distinct contracts are bounded, not lifetime suppression. */
reset_all();for(unsigned i=0;i<5;++i){offer(d1,sizeof(d1),255);uint64_t key=100+i;for(unsigned j=0;j<8;++j)candidate.data[candidate.len-12+j]=(key>>(8*j))&255;fix_crc();begin(3000+i*200);done(3005+i*200);assert(g_tx_last_forced_boundary_ms==3000+i*200);}assert(g_tx_closed_ack_count==4);offer(d1,sizeof(d1),255);uint64_t key=100;for(unsigned j=0;j<8;++j)candidate.data[candidate.len-12+j]=(key>>(8*j))&255;fix_crc();begin(4000);assert(g_tx_last_forced_boundary_ms==4000);done(4005);

reset_all();offer(d1,sizeof(d1),255);begin(5000);unsigned recoveries=g_tx_dma_recoveries;offer(d1,sizeof(d1),255);begin(5260);assert(g_tx_dma_recoveries==recoveries+1&&g_tx_last_forced_boundary_ms==5000&&g_tx_completed_service_credit_ms==0);assert(memcmp(g_tx_dma_item.data,d1,sizeof(d1))==0);done(5265);

/* One physical ACK burst serves both contracts, remembers each only after
 * a real close, and a same-ID retry never earns another promotion. */
reset_all();offer(d1,sizeof(d1),255);memcpy(candidate.data+candidate.len,d7,sizeof(d7));candidate.len+=sizeof(d7);candidate.uart_frames=2;begin(6000);assert(g_tx_closed_ack_count==2&&g_tx_last_forced_boundary_ms==6000);done(6010);assert(g_radio_tx_ok==2&&g_tx_dma_complete==1);
offer(d1,sizeof(d1),255);begin(6260);assert(g_tx_last_forced_boundary_ms==6000);done(6265);
offer(d7,sizeof(d7),255);begin(6520);assert(g_tx_last_forced_boundary_ms==6000);done(6525);
/* A remembered first receipt cannot hide a fresh second contract. */
offer(d1,sizeof(d1),255);memcpy(candidate.data+candidate.len,d8,sizeof(d8));candidate.len+=sizeof(d8);begin(6780);assert(g_tx_closed_ack_count==3&&g_tx_last_forced_boundary_ms==6780);done(6790);

/* A full recent-contract cache must retain every contract in this burst,
 * including an older hit that would otherwise be evicted by a new suffix. */
reset_all();for(unsigned i=0;i<4;++i){offer(d1,sizeof(d1),255);uint64_t k=100+i;for(unsigned j=0;j<8;++j)candidate.data[candidate.len-12+j]=(k>>(8*j))&255;fix_crc();begin(7000+200*i);done(7005+200*i);}
uint8_t mixed[3*sizeof(d1)];uint64_t keys[3]={102,100,104};
for(unsigned i=0;i<3;++i){offer(d1,sizeof(d1),255);for(unsigned j=0;j<8;++j)candidate.data[candidate.len-12+j]=(keys[i]>>(8*j))&255;fix_crc();memcpy(mixed+i*sizeof(d1),candidate.data,sizeof(d1));}
memcpy(candidate.data,mixed,sizeof(mixed));candidate.len=sizeof(mixed);candidate.uart_frames=3;begin(7800);done(7815);assert(g_tx_closed_ack_count==4);
for(unsigned i=0;i<3;++i){offer(d1,sizeof(d1),255);for(unsigned j=0;j<8;++j)candidate.data[candidate.len-12+j]=(keys[i]>>(8*j))&255;fix_crc();begin(8000+200*i);assert(g_tx_last_forced_boundary_ms==7800);done(8005+200*i);}
return 0;}

'''

class ReceiptPromotionDedupTests(unittest.TestCase):
    def test_actual_stale_receipt_burst_and_lost_first_retry_keep_bounded_recovery(self):
        source = (ROOT / 'Core/Src/radio.c').read_text()
        limit = next(line for line in source.splitlines() if line.startswith('#define RADIO_PACKET_SERVICE_CREDIT_CAP_MS '))
        code = function(source, 'static uint8_t radio_read_uleb128(')
        code += function(source[source.index('/* Only canonical packets are independent'):], 'static uint8_t radio_uart_independent_control(')
        code += source[source.index('static uint32_t radio_uart_air_ms(uint16_t len)\n{'):source.index('uint8_t radio_uart_tx_ready(void)')]
        code += function(source, 'uint32_t radio_uart_process_tx_with_budget(uint32_t budget_ms)\n{')
        code += function(source, 'void HAL_UART_TxCpltCallback(')
        code += function(source, 'void HAL_UART_ErrorCallback(')
        crc = function((ROOT / 'tests/radio_crc_reference.h').read_text(), 'uint32_t launchcore_crc32(')
        with tempfile.TemporaryDirectory() as directory:
            exe = str(Path(directory) / 'receipt-promotion-recovery')
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined', '-x', 'c', '-', '-o', exe], input=PREAMBLE+'\n'+limit+'\n'+crc+code+ACK_AND_NORMAL_FIXTURES+RECOVERY_CHECKS, text=True, check=True)
            subprocess.run([exe], check=True)
