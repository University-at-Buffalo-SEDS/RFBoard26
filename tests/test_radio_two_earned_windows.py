"""Two pre-earned RF idle windows: actual cold frames and bounded pressure."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_radio_funded_packet_boundary import PREAMBLE, function
ROOT = Path(__file__).resolve().parents[1]
COLD_FIXTURES = r'''
static const uint8_t f0[]={165,90,66,0,206,2,173,175,182,24,28,0,181,53,150,253,232,225,3,100,249,140,173,91,11,7,13,1,95,67,146,158,18,187,252,167,132,0,87,65,82,78,58,32,114,101,115,101,114,118,101,32,104,101,97,112,32,116,111,111,32,115,109,97,108,108,201,220,227,242};
static const uint8_t f1[]={165,90,46,0,83,68,84,1,206,217,193,253,6,136,3,106,0,231,210,234,182,183,51,159,231,1,238,218,132,102,248,181,196,85,249,140,173,91,132,154,150,127,104,227,160,231,234,14,27,48};
static const uint8_t f2[]={165,90,46,0,83,68,84,1,229,142,222,81,136,3,106,0,255,225,234,182,183,51,245,222,3,219,252,172,242,14,248,181,196,85,249,140,173,91,132,154,150,127,128,114,156,163,245,71,51,157};
static const uint8_t f3[]={165,90,62,0,83,68,84,1,149,252,188,138,11,136,1,101,24,238,211,234,182,183,51,203,12,150,253,232,225,3,249,140,173,91,0,0,0,0,0,0,0,0,229,59,236,65,0,0,0,0,0,0,0,0,0,0,0,0,34,123,25,64,253,174,50,34};
static const uint8_t f4[]={165,90,78,0,206,2,144,251,244,97,39,222,3,223,52,150,253,232,225,3,100,249,140,173,91,11,7,45,1,95,67,146,158,18,187,252,167,132,0,83,68,32,117,110,97,118,97,105,108,97,98,108,101,59,32,102,108,105,103,104,116,47,110,101,116,119,111,114,107,32,99,111,110,116,105,110,117,101,3,16,193,242};
static const uint8_t f5[]={165,90,50,0,12,1,1,8,237,236,234,182,183,51,211,32,164,134,212,212,6,20,3,68,2,2,100,66,44,90,241,216,214,133,95,67,146,158,18,187,252,167,71,192,73,39,238,100,103,224,95,100,127,85};
static const uint8_t f6[]={165,90,51,0,12,1,1,8,240,236,234,182,183,51,201,210,3,164,134,212,212,6,20,3,68,2,2,100,66,44,90,241,216,214,133,95,67,146,158,18,187,252,167,242,37,101,2,178,164,25,132,77,93,73,209};
static const uint8_t f7[]={165,90,51,0,12,1,1,8,171,237,234,182,183,51,161,137,3,164,134,212,212,6,20,3,68,2,2,100,66,44,90,241,216,214,133,95,67,146,158,18,187,252,167,161,255,82,91,232,147,224,198,77,254,65,46};

'''
COLD_CHECKS = r'''
int main(void){
 clock_ms=3386; memset(&candidate,0,sizeof(candidate));candidate.len=sizeof(f0);candidate.priority=0;memcpy(candidate.data,f0,sizeof(f0));offered=1;
 unsigned before_f0=g_tx_completed_service_credit_ms;
 if(radio_uart_process_tx_with_budget(2000)!=1){fprintf(stderr,"REFUSE f0 at3386 quiet=%u packet=%u active=%u\n",g_tx_quiet_until_ms,g_tx_packet_until_ms,g_tx_packet_active);return 2;}
 fprintf(stderr,"COLD frame=f0 type=ORDERED_MESSAGE ack=None start=3386 credit_before=%u credit_after_start=%u forced=%u\n",before_f0,g_tx_completed_service_credit_ms,g_tx_last_forced_boundary_ms);
 clock_ms=3392; HAL_UART_TxCpltCallback(g_huart);
 assert(g_tx_completed_service_credit_ms==27);
 clock_ms=3416; memset(&candidate,0,sizeof(candidate));candidate.len=sizeof(f1);candidate.priority=253;memcpy(candidate.data,f1,sizeof(f1));offered=1;
 unsigned before_f1=g_tx_completed_service_credit_ms;
 if(radio_uart_process_tx_with_budget(2000)!=1){fprintf(stderr,"REFUSE f1 at3416 quiet=%u packet=%u active=%u\n",g_tx_quiet_until_ms,g_tx_packet_until_ms,g_tx_packet_active);return 2;}
 fprintf(stderr,"COLD frame=f1 type=HEARTBEAT ack=None start=3416 credit_before=%u credit_after_start=%u forced=%u\n",before_f1,g_tx_completed_service_credit_ms,g_tx_last_forced_boundary_ms);
 clock_ms=3420; HAL_UART_TxCpltCallback(g_huart);
 assert(g_tx_completed_service_credit_ms==48);
 clock_ms=3676; memset(&candidate,0,sizeof(candidate));candidate.len=sizeof(f2);candidate.priority=253;memcpy(candidate.data,f2,sizeof(f2));offered=1;
 unsigned before_f2=g_tx_completed_service_credit_ms;
 if(radio_uart_process_tx_with_budget(2000)!=1){fprintf(stderr,"REFUSE f2 at3676 quiet=%u packet=%u active=%u\n",g_tx_quiet_until_ms,g_tx_packet_until_ms,g_tx_packet_active);return 2;}
 fprintf(stderr,"COLD frame=f2 type=HEARTBEAT ack=None start=3676 credit_before=%u credit_after_start=%u forced=%u\n",before_f2,g_tx_completed_service_credit_ms,g_tx_last_forced_boundary_ms);
 clock_ms=3680; HAL_UART_TxCpltCallback(g_huart);
 assert(g_tx_completed_service_credit_ms==69);
 clock_ms=3779; memset(&candidate,0,sizeof(candidate));candidate.len=sizeof(f3);candidate.priority=0;memcpy(candidate.data,f3,sizeof(f3));offered=1;
 unsigned before_f3=g_tx_completed_service_credit_ms;
 if(radio_uart_process_tx_with_budget(2000)!=1){fprintf(stderr,"REFUSE f3 at3779 quiet=%u packet=%u active=%u\n",g_tx_quiet_until_ms,g_tx_packet_until_ms,g_tx_packet_active);return 2;}
 fprintf(stderr,"COLD frame=f3 type=IMU_DATA ack=None start=3779 credit_before=%u credit_after_start=%u forced=%u\n",before_f3,g_tx_completed_service_credit_ms,g_tx_last_forced_boundary_ms);
 clock_ms=3785; HAL_UART_TxCpltCallback(g_huart);
 assert(g_tx_completed_service_credit_ms==95);
 clock_ms=3805; memset(&candidate,0,sizeof(candidate));candidate.len=sizeof(f4);candidate.priority=0;memcpy(candidate.data,f4,sizeof(f4));offered=1;
 unsigned before_f4=g_tx_completed_service_credit_ms;
 if(radio_uart_process_tx_with_budget(2000)!=1){fprintf(stderr,"REFUSE f4 at3805 quiet=%u packet=%u active=%u\n",g_tx_quiet_until_ms,g_tx_packet_until_ms,g_tx_packet_active);return 2;}
 fprintf(stderr,"COLD frame=f4 type=WARNING ack=None start=3805 credit_before=%u credit_after_start=%u forced=%u\n",before_f4,g_tx_completed_service_credit_ms,g_tx_last_forced_boundary_ms);
 clock_ms=3812; HAL_UART_TxCpltCallback(g_huart);
 assert(g_tx_completed_service_credit_ms==110);
 clock_ms=3837; memset(&candidate,0,sizeof(candidate));candidate.len=sizeof(f5);candidate.priority=255;memcpy(candidate.data,f5,sizeof(f5));offered=1;
 unsigned before_f5=g_tx_completed_service_credit_ms;
 if(radio_uart_process_tx_with_budget(2000)!=1){fprintf(stderr,"REFUSE f5 at3837 quiet=%u packet=%u active=%u\n",g_tx_quiet_until_ms,g_tx_packet_until_ms,g_tx_packet_active);return 2;}
 fprintf(stderr,"COLD frame=f5 type=ReliableAck ack=16170003961121194055 start=3837 credit_before=%u credit_after_start=%u forced=%u\n",before_f5,g_tx_completed_service_credit_ms,g_tx_last_forced_boundary_ms);
 clock_ms=3842; HAL_UART_TxCpltCallback(g_huart);
 assert(g_tx_completed_service_credit_ms==110);
 clock_ms=3897; memset(&candidate,0,sizeof(candidate));candidate.len=sizeof(f6);candidate.priority=255;memcpy(candidate.data,f6,sizeof(f6));offered=1;
 unsigned before_f6=g_tx_completed_service_credit_ms;
 if(radio_uart_process_tx_with_budget(2000)!=1){fprintf(stderr,"REFUSE f6 at3897 quiet=%u packet=%u active=%u\n",g_tx_quiet_until_ms,g_tx_packet_until_ms,g_tx_packet_active);return 2;}
 fprintf(stderr,"COLD frame=f6 type=ReliableAck ack=9518820371875571186 start=3897 credit_before=%u credit_after_start=%u forced=%u\n",before_f6,g_tx_completed_service_credit_ms,g_tx_last_forced_boundary_ms);
 clock_ms=3902; HAL_UART_TxCpltCallback(g_huart);
 assert(g_tx_completed_service_credit_ms==55);
 clock_ms=3959; memset(&candidate,0,sizeof(candidate));candidate.len=sizeof(f7);candidate.priority=255;memcpy(candidate.data,f7,sizeof(f7));offered=1;
 unsigned before_f7=g_tx_completed_service_credit_ms;
 if(radio_uart_process_tx_with_budget(2000)!=1){fprintf(stderr,"REFUSE f7 at3959 quiet=%u packet=%u active=%u\n",g_tx_quiet_until_ms,g_tx_packet_until_ms,g_tx_packet_active);return 2;}
 fprintf(stderr,"COLD frame=f7 type=ReliableAck ack=14330616640466780065 start=3959 credit_before=%u credit_after_start=%u forced=%u\n",before_f7,g_tx_completed_service_credit_ms,g_tx_last_forced_boundary_ms);
 clock_ms=3963; HAL_UART_TxCpltCallback(g_huart);
 assert(g_tx_completed_service_credit_ms==0);
 fprintf(stderr,"FINAL third_forced=%u credit=%u\n",g_tx_last_forced_boundary_ms,g_tx_completed_service_credit_ms);
#ifdef EXPECT_THIRD_CLOSE
assert(g_tx_last_forced_boundary_ms==3959);
#else
assert(g_tx_last_forced_boundary_ms==3897);
#endif

clock_ms=4020;memset(&candidate,0,sizeof(candidate));candidate.len=sizeof(f7);candidate.priority=255;memcpy(candidate.data,f7,sizeof(f7));uint64_t fourth=UINT64_C(999999999);for(unsigned i=0;i<8;++i)candidate.data[candidate.len-12+i]=(fourth>>(8*i))&255;uint32_t crc=launchcore_crc32(candidate.data+4,candidate.len-8);for(unsigned i=0;i<4;++i)candidate.data[candidate.len-4+i]=(crc>>(8*i))&255;offered=1;assert(radio_uart_process_tx_with_budget(2000)==1);assert(g_tx_last_forced_boundary_ms==3959&&g_tx_completed_service_credit_ms==0);
return 0;}

'''
PRESSURE_FIXTURES = r'''
static const uint8_t f7[]={165,90,51,0,12,1,1,8,171,237,234,182,183,51,161,137,3,164,134,212,212,6,20,3,68,2,2,100,66,44,90,241,216,214,133,95,67,146,158,18,187,252,167,161,255,82,91,232,147,224,198,77,254,65,46};
static const uint8_t schema_metadata[]={165,90,192,0,83,68,84,3,234,69,154,45,0,0,18,0,74,1,10,185,24,0,205,205,1,219,252,172,242,14,132,0,3,0,0,0,12,0,0,0,202,0,0,0,0,13,0,0,0,83,69,68,83,78,69,84,95,69,82,82,79,82,0,0,0,0,200,0,0,0,0,17,0,0,0,83,69,68,83,78,69,84,95,84,73,77,69,95,83,89,78,67,0,0,0,0,201,0,0,0,0,17,0,0,0,83,69,68,83,78,69,84,95,68,73,83,67,79,86,69,82,89,0,0,0,0,100,0,0,0,0,7,0,0,0,83,68,95,67,65,82,68,0,0,0,0,5,176,149,1,0,22,0,0,0,65,86,95,66,65,89,95,85,78,68,69,82,71,76,79,87,95,79,87,78,69,82,0,0,0,0,49,156,249,2,0,14,0,0,0,65,67,77,28,145,219};
static const uint8_t large_ordinary[]={ 165,90,184,0,136,1,157,233,246,65,160,1,1,181,53,169,212,158,205,6,209,153,166,88,2,182,83,29,99,143,92,156,33,101,81,59,71,151,246,79,231,250,55,245,118,224,117,109,213,170,163,10,46,188,119,157,234,65,7,93,215,160,19,85,187,244,107,127,17,19,105,217,152,245,226,245,180,141,120,238,189,50,92,54,126,88,76,54,234,225,240,193,192,159,4,220,99,189,192,185,128,87,107,209,10,164,161,8,98,156,215,248,185,254,213,208,139,142,113,147,34,184,135,72,86,34,216,95,239,224,79,93,137,100,106,18,178,161,234,241,232,87,3,9,45,41,53,171,30,180,195,140,75,21,87,85,73,211,144,87,121,57,185,12,164,149,72,186,185,136,153,224,159,42,162,241,250,123,47,135,2,164,129,178,64,61,70,86 };

'''
PRESSURE_CHECKS = r'''

static void reset_all(void){g_tx_packet_active=g_tx_air_budget_active=g_tx_dma_busy=g_tx_dma_preserves_rx=0;g_tx_last_forced_boundary_ms=g_tx_completed_service_credit_ms=g_tx_pending_service_ms=0;g_tx_closed_ack_count=g_tx_closed_ack_cursor=0;memset(g_tx_closed_ack_ids,0,sizeof(g_tx_closed_ack_ids));g_radio_tx_ok=g_tx_dma_complete=0;offered=0;hal_status=HAL_OK;}
static void offer(const uint8_t *bytes,unsigned len,unsigned priority){memset(&candidate,0,sizeof(candidate));candidate.len=len;candidate.priority=priority;memcpy(candidate.data,bytes,len);offered=1;}
static void fix_crc(void){unsigned n=candidate.len;unsigned crc=launchcore_crc32(candidate.data+4,n-8);for(unsigned i=0;i<4;++i)candidate.data[n-4+i]=(crc>>(8*i))&255;}
static void large_frame(unsigned index){offer(large_ordinary,sizeof(large_ordinary),0);size_t at=6;uint64_t ignore;assert(radio_read_uleb128(candidate.data,candidate.len,&at,&ignore));assert(radio_read_uleb128(candidate.data,candidate.len,&at,&ignore));assert(!(candidate.data[at]&128));candidate.data[at]=index;fix_crc();assert(candidate.len>=176);}
int main(void){
 for(unsigned period=100;period<=100;period+=50)for(unsigned warm=0;warm<2;++warm){
  reset_all();unsigned earned=0,spent=0;
  if(warm){for(unsigned i=0;i<2;++i){clock_ms=1000+i*80;large_frame(42+i);assert(radio_uart_process_tx_with_budget(2000));earned+=radio_uart_air_ms(candidate.len);clock_ms+=20;HAL_UART_TxCpltCallback(g_huart);}}
  unsigned base=warm?2000:0,ai=0,ao=0,ni=0,no=0,mi=0,mo=0,finish=0,kind=0,finish_kind=0,maxn=0,maxa=0,maxm=0,peak=0;unsigned at[200],nt[50],mt[10];
  for(unsigned t=0;t<=12000;++t){unsigned now=base+t;
   if(g_tx_dma_busy&&now>=finish){clock_ms=now;HAL_UART_TxCpltCallback(g_huart);if(finish_kind==2)earned+=radio_uart_air_ms(g_tx_dma_item.len);}
   if(t<=8000&&t%period==0)at[ai++]=t;
   if(t<=8000&&t%200==0)nt[ni++]=t;
   if(t<=8000&&t%1000==0)mt[mi++]=t;
   unsigned pending=ai-ao+ni-no+mi-mo;if(pending>peak)peak=pending;
   clock_ms=now;if(g_tx_dma_busy||radio_uart_air_busy())continue;
   if(ao<ai&&(g_tx_independent_streak<7U||(no==ni&&mo==mi))){kind=1;offer(f7,sizeof(f7),255);uint64_t key=1000000+ao;for(unsigned i=0;i<8;++i)candidate.data[candidate.len-12+i]=(key>>(8*i))&255;fix_crc();}
   else if(no<ni&&(mo==mi||nt[no]<=mt[mo])){kind=2;large_frame(no);}
   else if(mo<mi){kind=3;offer(schema_metadata,sizeof(schema_metadata),254);}else continue;
   unsigned before=g_tx_completed_service_credit_ms;
   if(radio_uart_process_tx_with_budget(2000)){
    unsigned wire=(candidate.len*10000U+115199U)/115200U;finish=now+wire;finish_kind=kind;
    unsigned after=g_tx_completed_service_credit_ms;if(after<before)spent+=before-after;
    if(kind==1){unsigned age=t-at[ao++]+wire;if(age>maxa)maxa=age;}
    if(kind==2){unsigned age=t-nt[no++]+wire;if(age>maxn)maxn=age;}
    if(kind==3){unsigned age=t-mt[mo++]+wire;if(age>maxm)maxm=age;}
    assert(spent<=earned&&g_tx_completed_service_credit_ms<=110);
   }
  }
  fprintf(stderr,"THREE_LANE period=%u warm=%u normal=%u/%u ACK=%u/%u meta=%u/%u maxnormal=%u maxACK=%u maxmeta=%u peak=%u earned=%u spent=%u\n",period,warm,no,ni,ao,ai,mo,mi,maxn,maxa,maxm,peak,earned,spent);
  assert(no==ni&&ao==ai&&mo==mi&&maxn<=1500&&maxa<=500&&maxm<=3000&&peak<=12);
 }
 return 0;
}

'''

def run_actual_source(fixtures, checks):
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
        exe = str(Path(directory) / 'two-earned-windows')
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined', '-DEXPECT_THIRD_CLOSE=1', '-x', 'c', '-', '-o', exe], input=PREAMBLE + '\n' + limit + '\n' + crc + code + fixtures + checks, text=True, check=True)
        subprocess.run([exe], check=True)

class TwoEarnedWindowTests(unittest.TestCase):
    def test_zero_initial_bank_cold_three_ack_cluster_and_fourth_denial(self):
        run_actual_source(COLD_FIXTURES, COLD_CHECKS)

    def test_large_ordinary_metadata_and_unique_ack_pressure_cold_and_earned(self):
        # 20ACK/s+large-data diagnostic overloads baseline77 too; it remains
        # separate negative evidence. This baseline-qualified10ACK/s case
        # preserves original queue12 and500/1500/3000ms bounds.
        run_actual_source(PRESSURE_FIXTURES, PRESSURE_CHECKS)
