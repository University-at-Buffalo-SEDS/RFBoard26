from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class CanTxQueueTests(unittest.TestCase):
    def test_owned_buffers_fragment_order_backpressure_and_recovery(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)
            (p/'can_bus.h').write_text('''#pragma once
#include <stdint.h>
#include <stddef.h>
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_BUSY 2
HAL_StatusTypeDef can_bus_send_bytes(const uint8_t*,size_t,uint32_t);
''')
            (p/'can_tx_queue.h').write_text((ROOT/'Core/Inc/can_tx_queue.h').read_text())
            (p/'can_link.h').write_text((ROOT/'Core/Inc/can_link.h').read_text())
            (p/'board_clock_us.h').write_text('''#pragma once
#include <stdint.h>
extern uint64_t now_us;
static inline uint64_t board_now_us(void){return now_us;}
''')
            (p/'main.h').write_text('''#include <stdint.h>
extern uint32_t irq;
static inline uint32_t __get_PRIMASK(void){return irq;}
static inline void __disable_irq(void){irq=1;}
static inline void __set_PRIMASK(uint32_t n){irq=n;}
uint32_t HAL_GetTick(void);
''')
            (p/'test.c').write_text(r'''
#include "can_tx_queue.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
uint32_t irq;
static uint32_t tick;
static unsigned free_slots,allocations,frees,failed_alloc,hal_error,count,last_protected;
static uint8_t wire[4096][64];static uint32_t ids[4096];static unsigned lens[4096];
extern volatile uint32_t g_can_tx_queue_bytes,g_can_tx_pending,g_can_tx_rejected,g_can_tx_expired,g_can_tx_submitted;
uint32_t HAL_GetTick(void){return tick;}
uint64_t now_us;static uint32_t latest_buffer=1U;
uint32_t can_bus_latest_tx_buffer(void){assert(irq);return latest_buffer;}
void *telemetry_can_tx_allocate_priority(size_t n,uint8_t protected_control){assert(protected_control<=1);last_protected=protected_control;assert(!irq);if(failed_alloc)return NULL;allocations++;return malloc(n);}
void telemetryFree(void *p){assert(!irq);frees++;free(p);}
HAL_StatusTypeDef can_bus_send_bytes(const uint8_t *p,size_t n,uint32_t id){
 /* Link format v2: exact CAN FD lengths, no payload header, kind in the id. */
 assert(irq && n>=1 && n<=64 && can_link_dlc_len(n)==n && id<0x200);
 assert(can_link_payload_len(id,p,n)>0);
 if(hal_error)return HAL_ERROR;
 if(!free_slots)return HAL_BUSY;
 free_slots--;assert(count<4096);memset(wire[count],0,64);memcpy(wire[count],p,n);lens[count]=(unsigned)n;ids[count++]=id;return HAL_OK;
}
static can_link_slot_t rx_slots[2];
/* Reassemble wire frames [from,to) (one packet) through the receive path. */
static size_t reassemble(unsigned from,unsigned to,uint8_t *out){
 size_t got=0;
 for(unsigned f=from;f<to;f++){
  const uint8_t *pkt=NULL;size_t n=0;
  int r=can_link_rx(rx_slots,2,ids[f],wire[f],lens[f],0,&pkt,&n);
  if(f+1<to){assert(r==CAN_LINK_RX_PARTIAL);continue;}
  assert(r==CAN_LINK_RX_COMPLETE);memcpy(out,pkt,n);got=n;
 }
 return got;
}
static unsigned kind(unsigned f){return ids[f]&CAN_LINK_KIND_MASK;}
static void interrupt(void){unsigned a=allocations,f=frees;irq=1;can_tx_queue_pump();assert(irq && allocations==a && frees==f);irq=0;}
static void drain(void){while(g_can_tx_pending){free_slots=3;interrupt();can_tx_queue_service();}}
int main(void){
 uint8_t data[128];for(unsigned i=0;i<128;i++)data[i]=i;
 assert(can_tx_queue_submit(NULL,1,0)==HAL_ERROR);
 assert(can_tx_queue_submit(data,0,0)==HAL_ERROR);
 assert(can_tx_queue_submit(data,129,0)==HAL_ERROR);
 assert(can_tx_queue_submit(data,128,0x105)==HAL_OK);
 assert(!last_protected);assert(g_can_tx_pending==1 && count==0);memset(data,99,sizeof(data));
 free_slots=1;interrupt();assert(count==1 && frees==0 && g_can_tx_pending==1);
 hal_error=1;free_slots=3;interrupt();assert(count==1);
 hal_error=0;interrupt();assert(count==2 && g_can_tx_pending==0 && frees==0);
 /* Two whole 64-byte frames: FIRST then LAST, band and board unchanged. */
 assert(ids[0]==can_link_id(0x105,CAN_LINK_FIRST) && ids[0]==0x12AU && lens[0]==64);
 assert(ids[1]==can_link_id(0x105,CAN_LINK_LAST) && ids[1]==0x12CU && lens[1]==64);
 for(unsigned f=0;f<2;f++)for(unsigned i=0;i<64;i++)assert(wire[f][i]==f*64+i);
 {uint8_t out[128];assert(reassemble(0,2,out)==128);for(unsigned i=0;i<128;i++)assert(out[i]==i);}
 can_tx_queue_service();assert(frees==1 && !g_can_tx_queue_bytes);
 // The SDK's outbound side-frame cap fills exactly two physical frames.
 assert(CAN_TX_ALIGNED_FRAME_BYTES==128 && CAN_TX_FRAGMENT_PAYLOAD_BYTES==64);
 unsigned aligned_first=count;free_slots=3;
 assert(can_tx_queue_submit(data,CAN_TX_ALIGNED_FRAME_BYTES,0x105)==HAL_OK);
 assert(count==aligned_first+2 && !g_can_tx_pending);
 for(unsigned f=aligned_first;f<count;f++){
  assert(lens[f]==64);for(unsigned j=0;j<64;j++)assert(wire[f][j]==99);
 }
 can_tx_queue_service();assert(allocations==frees && !g_can_tx_queue_bytes);
 // Hard byte/count limit: repeated refusal cannot leak an allocation.
 free_slots=0;unsigned accepted=0;
 while(can_tx_queue_submit(data,128,0x105)==HAL_OK)accepted++;
 assert(accepted>1 && accepted<=48 && g_can_tx_queue_bytes<=6144);
 unsigned a=allocations,pending=g_can_tx_pending;
 for(unsigned i=0;i<100;i++)assert(can_tx_queue_submit(data,128,0x105)==HAL_BUSY);
 assert(allocations==a && g_can_tx_pending==pending);
 drain();assert(!g_can_tx_queue_bytes && allocations==frees);
 // Ordered messages keep each packet's fragments consecutive.
 for(unsigned i=4;i<count;i++){
  if(kind(i)==CAN_LINK_LAST||kind(i)==(CAN_LINK_LAST|CAN_LINK_PAD))
   assert(kind(i-1)==CAN_LINK_FIRST && (ids[i]&CAN_LINK_STREAM_MASK)==(ids[i-1]&CAN_LINK_STREAM_MASK));
 }
 failed_alloc=1;free_slots=0;assert(can_tx_queue_submit(data,1,0)==HAL_BUSY);failed_alloc=0;
 // Expire a partly emitted packet across timer wrap; no stale remainder sent.
 tick=UINT32_MAX-500;free_slots=1;assert(can_tx_queue_submit(data,128,0)==HAL_OK);
 unsigned before=count;tick=499;free_slots=3;interrupt();assert(count==before && g_can_tx_expired==1);
 can_tx_queue_service();assert(!g_can_tx_queue_bytes && allocations==frees);
 free_slots=0;assert(can_tx_queue_submit(data,128,0)==HAL_OK);can_tx_queue_reset();
 assert(!g_can_tx_queue_bytes && !g_can_tx_pending && allocations==frees);
 // Priority identity survives the copy boundary; bulk cannot borrow it.
 free_slots=3;assert(can_tx_queue_submit(data,59,0x004)==HAL_OK);assert(last_protected);
 free_slots=3;assert(can_tx_queue_submit(data,59,0x104)==HAL_OK);assert(!last_protected);
 // Saturated ordinary traffic must retain metadata and receipt capacity.
 // A canonical receipt may interrupt a partially sent bulk packet; the
 // original packet then resumes at the same fragment/sequence, once only.
 unsigned source=can_link_board(ids[0]), data_id=0x100|source, meta_id=0x80|source;
 unsigned first=count;free_slots=1;
 assert(can_tx_queue_submit(data,128,data_id)==HAL_OK);
 assert(count==first+1 && kind(first)==CAN_LINK_FIRST);
 free_slots=0;unsigned bulk=0,metadata=0;
 while(can_tx_queue_submit(data,128,data_id)==HAL_OK)bulk++;
 while(can_tx_queue_submit(data,128,meta_id)==HAL_OK)metadata++;
 assert(bulk>0 && metadata>0);
 assert(can_tx_queue_submit(data,20,source)==HAL_OK);
 free_slots=1;interrupt();
 /* The receipt is one SINGLE frame in the control band, above the data band. */
 assert(count==first+2 && ids[first+1]==can_link_id(source,CAN_LINK_SINGLE) && ids[first+1]<ids[first]);
 free_slots=3;interrupt();
 assert(ids[first+2]==can_link_id(data_id,CAN_LINK_LAST));
 {uint8_t out[128];const unsigned order[3]={first,first+1,first+2};
  /* Interleaved streams reassemble independently: data FIRST, control, data LAST. */
  const uint8_t *pkt;size_t n;
  assert(can_link_rx(rx_slots,2,ids[order[0]],wire[order[0]],lens[order[0]],0,&pkt,&n)==CAN_LINK_RX_PARTIAL);
  assert(can_link_rx(rx_slots,2,ids[order[1]],wire[order[1]],lens[order[1]],0,&pkt,&n)==CAN_LINK_RX_COMPLETE && n==20);
  assert(can_link_rx(rx_slots,2,ids[order[2]],wire[order[2]],lens[order[2]],0,&pkt,&n)==CAN_LINK_RX_COMPLETE && n==128);
  memcpy(out,pkt,n);for(unsigned i=0;i<128;i++)assert(out[i]==data[i]);}
 drain();assert(!g_can_tx_queue_bytes && allocations==frees);
 // Repeated small packets exercise sequence and queue reuse/wrap.
 for(unsigned i=0;i<300;i++){free_slots=3;assert(can_tx_queue_submit(data,1,0)==HAL_OK);}
 // Every supported size round-trips through the receive path; padding is
 // marked in the id and the last padding byte, never in a payload header.
 for(unsigned len=1;len<=128;len++){
  unsigned first=count;for(unsigned n=0;n<len;n++)data[n]=(uint8_t)(n*7+len);
  free_slots=3;assert(can_tx_queue_submit(data,len,0x101)==HAL_OK);
  assert(count-first==(len+63)/64);
  for(unsigned f=first;f<count;f++)assert(can_link_board(ids[f])==1 && (ids[f]&CAN_LINK_BAND_MASK)==0x100);
  const unsigned tail=len-(count-first-1)*64;
  assert(lens[count-1]==can_link_dlc_len(tail));
  assert(((ids[count-1]&CAN_LINK_PAD)!=0)==(can_link_dlc_len(tail)!=tail));
  uint8_t out[128];assert(reassemble(first,count,out)==len && !memcmp(out,data,len));
 }
 assert(allocations==frees);
 // Time sync TX stamp: the watched packet's last fragment is stamped at its
 // hardware buffer's completion, not at submit or at an unrelated buffer.
 {const uint8_t *got=NULL;size_t got_len=0;uint64_t sent=0;
  assert(!can_tx_queue_take_watched_sent(&got,&got_len,&sent));
  for(unsigned n=0;n<70;n++)data[n]=(uint8_t)(200+n);
  can_tx_queue_watch_next();free_slots=1;latest_buffer=1U<<2;
  assert(can_tx_queue_submit(data,70,0x101)==HAL_OK);          /* two fragments, one sent */
  memset(data,0,70);
  can_tx_queue_note_complete(1U<<2);assert(!can_tx_queue_take_watched_sent(&got,&got_len,&sent));
  latest_buffer=1U<<1;free_slots=1;interrupt();                /* last fragment in buffer 1 */
  now_us=777;can_tx_queue_note_complete(1U<<2);assert(!can_tx_queue_take_watched_sent(&got,&got_len,&sent));
  now_us=888;can_tx_queue_note_complete(1U<<1);
  assert(can_tx_queue_take_watched_sent(&got,&got_len,&sent)&&sent==888&&got_len==70);
  for(unsigned n=0;n<70;n++)assert(got[n]==(uint8_t)(200+n));
  assert(!can_tx_queue_take_watched_sent(&got,&got_len,&sent));   /* taken once */
  can_tx_queue_service();assert(allocations==frees);}
 // The watch recovers from every path that loses the watched packet, so
 // time sync keeps its transmit stamps without a reboot.
 {const uint8_t *got=NULL;size_t got_len=0;uint64_t sent=0;unsigned first;
  extern volatile uint32_t g_can_tx_watch_abandoned;
#define STAMPED(n) do{first=count;can_tx_queue_watch_next();free_slots=3;latest_buffer=1U<<0;\
  assert(can_tx_queue_submit(data,(n),0x101)==HAL_OK);assert(count>first);now_us=4242;\
  can_tx_queue_note_complete(1U<<0);assert(can_tx_queue_take_watched_sent(&got,&got_len,&sent));\
  assert(sent==4242&&got_len==(n));can_tx_queue_service();}while(0)
  STAMPED(20);
  // 1. Watched packet expires at the queue head (never reached hardware).
  tick=10000;free_slots=0;can_tx_queue_watch_next();
  assert(can_tx_queue_submit(data,20,0x101)==HAL_OK);
  tick+=1000;free_slots=3;first=count;interrupt();assert(count==first);
  can_tx_queue_service();assert(g_can_tx_watch_abandoned==1);
  assert(!can_tx_queue_take_watched_sent(&got,&got_len,&sent));
  STAMPED(21);
  // 2. Bus-off recovery drops the queued watched packet.
  free_slots=0;can_tx_queue_watch_next();assert(can_tx_queue_submit(data,20,0x101)==HAL_OK);
  can_tx_queue_reset();assert(g_can_tx_watch_abandoned==2);
  STAMPED(22);
  // 3. Bus-off recovery aborts the hardware buffer holding the last fragment.
  can_tx_queue_watch_next();free_slots=3;latest_buffer=1U<<1;
  assert(can_tx_queue_submit(data,20,0x101)==HAL_OK);
  can_tx_queue_reset();assert(g_can_tx_watch_abandoned==3);
  now_us=1;can_tx_queue_note_complete(1U<<1);           /* late, aborted buffer */
  assert(!can_tx_queue_take_watched_sent(&got,&got_len,&sent));
  STAMPED(23);
  // 4. A completion that never arrives is abandoned after the queue age.
  can_tx_queue_watch_next();free_slots=3;latest_buffer=1U<<2;
  assert(can_tx_queue_submit(data,20,0x101)==HAL_OK);
  tick+=999;can_tx_queue_watch_next();                   /* still in flight */
  free_slots=3;latest_buffer=1U<<0;assert(can_tx_queue_submit(data,19,0x101)==HAL_OK);
  now_us=5;can_tx_queue_note_complete(1U<<0);           /* unrelated buffer */
  assert(!can_tx_queue_take_watched_sent(&got,&got_len,&sent));
  can_tx_queue_watch_disarm();tick+=1;
  STAMPED(24);assert(g_can_tx_watch_abandoned==4);
  // 5. A refused submit consumes the request; the next packet is not watched.
  can_tx_queue_watch_next();failed_alloc=1;free_slots=3;
  assert(can_tx_queue_submit(data,20,0x101)==HAL_BUSY);failed_alloc=0;
  latest_buffer=1U<<0;assert(can_tx_queue_submit(data,20,0x101)==HAL_OK);
  can_tx_queue_note_complete(1U<<0);assert(!can_tx_queue_take_watched_sent(&got,&got_len,&sent));
  STAMPED(25);
  // 6. An oversize packet is not watched and does not pass the watch on.
  can_tx_queue_watch_next();free_slots=3;assert(can_tx_queue_submit(data,100,0x101)==HAL_OK);
  assert(can_tx_queue_submit(data,20,0x101)==HAL_OK);
  can_tx_queue_note_complete(7U);assert(!can_tx_queue_take_watched_sent(&got,&got_len,&sent));
  // 7. A send refused before the queue: disarm keeps the next packet unwatched.
  can_tx_queue_watch_next();can_tx_queue_watch_disarm();
  free_slots=3;assert(can_tx_queue_submit(data,20,0x101)==HAL_OK);
  can_tx_queue_note_complete(7U);assert(!can_tx_queue_take_watched_sent(&got,&got_len,&sent));
  STAMPED(26);
  can_tx_queue_service();assert(allocations==frees&&!g_can_tx_queue_bytes);}
}
''')
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                            '-I',str(p),str(ROOT/'Core/Src/can_tx_queue.c'),str(p/'test.c'),'-o',str(p/'test')],check=True)
            subprocess.run([str(p/'test')],check=True)

    def test_queue_allocation_preserves_control_reserve(self):
        source = (ROOT/'Core/Src/telemetry_hooks.c').read_text()
        start = source.index('void *telemetry_can_tx_allocate(')
        end = source.index('void telemetryFree(', start)
        harness = r'''
#include <stddef.h>
#include <stdint.h>
#include <assert.h>
typedef unsigned long ULONG;
typedef struct {ULONG available;} TX_BYTE_POOL;
#define TX_NULL NULL
#define TX_SUCCESS 0
static TX_BYTE_POOL small={4200};
static TX_BYTE_POOL *rust_byte_pool_external=&small;
static unsigned allocations,denied,protected_checks;
static int tx_byte_pool_info_get(TX_BYTE_POOL *p,void*a,ULONG*v,void*b,void*c,void*d,void*e){
 (void)a;(void)b;(void)c;(void)d;(void)e;*v=p->available;return 0;
}
static int telemetry_tlsf_admit(size_t additional,size_t largest){assert(additional==largest+512);return !denied;}
static int telemetry_tlsf_admit_can_control(size_t n){assert(n && n<=160);protected_checks++;return !denied;}
static void *telemetryMalloc(size_t n){(void)n;allocations++;return &small;}
''' + source[start:end] + r'''
int main(void){
#ifdef TELEMETRY_USE_TLSF
 assert(telemetry_can_tx_allocate(128));denied=1;
 assert(!telemetry_can_tx_allocate(128));assert(allocations==1);
 assert(!telemetry_can_tx_allocate_priority(128,1));assert(protected_checks==1);
 denied=0;assert(telemetry_can_tx_allocate_priority(128,1));assert(protected_checks==2);
 assert(telemetry_can_tx_allocate_priority(128,0));assert(protected_checks==2);
 assert(telemetry_can_tx_allocate_priority(161,1));assert(protected_checks==2);
#else
 assert(!telemetry_can_tx_allocate(128));assert(!allocations);
 assert(!telemetry_can_tx_allocate_priority(128,0));
 assert(telemetry_can_tx_allocate_priority(128,1));allocations=0;
 small.available=2468;assert(!telemetry_can_tx_allocate_priority(128,0));
 assert(telemetry_can_tx_allocate_priority(128,1));allocations=0;small.available=4200;
 small.available=4224;assert(telemetry_can_tx_allocate(128));assert(allocations==1);
 small.available=1;assert(!telemetry_can_tx_allocate(128));
 rust_byte_pool_external=NULL;assert(!telemetry_can_tx_allocate(128));
#endif
}
'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'test.c').write_text(harness)
            for defines in [[],['-DTELEMETRY_USE_TLSF']]:
                subprocess.run(['cc','-std=c11',*defines,str(p/'test.c'),'-o',str(p/'test')],check=True)
                subprocess.run([str(p/'test')],check=True)

if __name__=='__main__':unittest.main()
