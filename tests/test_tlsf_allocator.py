import subprocess
import tempfile
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]

class TlsfAllocatorTests(unittest.TestCase):
    def test_board_adapter_churn_exhaustion_and_recovery(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            (p/'main.h').write_text('''#include <stdint.h>
extern uint32_t mock_irq;
static inline uint32_t __get_PRIMASK(void){return mock_irq;}
static inline void __disable_irq(void){mock_irq=1;}
static inline void __set_PRIMASK(uint32_t m){mock_irq=m;}
void Error_Handler(void);
''')
            (p/'tx_api.h').write_text('''#ifndef MOCK_TX_H
#define MOCK_TX_H
#include <stddef.h>
typedef unsigned UINT;
typedef unsigned long ULONG;
typedef struct { unsigned char *base; size_t used, bytes; } TX_BYTE_POOL;
#define TX_SUCCESS 0
#define TX_NO_MEMORY 1
#define TX_NO_WAIT 0
#define TX_NULL NULL
UINT tx_byte_allocate(TX_BYTE_POOL*,void**,ULONG,UINT);
UINT tx_byte_pool_info_get(TX_BYTE_POOL*,void*,ULONG*,ULONG*,void*,void*,void*);
#endif
''')
            (p/'test.c').write_text(r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "telemetry_tlsf.h"
#include "tlsf.h"
#include "telemetry_tlsf.c"
uint32_t mock_irq;
static unsigned calls, fail_init;
void Error_Handler(void){abort();}
UINT tx_byte_allocate(TX_BYTE_POOL*p,void**out,ULONG n,UINT wait){
 assert(wait==TX_NO_WAIT);calls++;
 size_t size=(n+7)&~(size_t)7;
 if(fail_init || size+16>p->bytes-p->used)return TX_NO_MEMORY;
 *out=p->base+p->used;p->used+=size+16;return TX_SUCCESS;
}
UINT tx_byte_pool_info_get(TX_BYTE_POOL*p,void*n,ULONG*a,ULONG*f,void*x,void*y,void*z){
 (void)n;(void)x;(void)y;(void)z;
 if(a)*a=p->bytes-p->used;if(f)*f=1;return TX_SUCCESS;
}
extern volatile uint32_t g_telemetry_tlsf_active,g_telemetry_tlsf_init_failed;
extern volatile uint32_t g_telemetry_tlsf_snapshot_count;
extern volatile uint32_t g_telemetry_tlsf_live_bytes,g_telemetry_tlsf_peak_bytes;
extern volatile uint32_t g_telemetry_tlsf_free_bytes,g_telemetry_tlsf_largest_free;
static uint64_t arenas[3][4096];
static void *ordinary_only(size_t n){
 void *p=tlsf_memalign(allocator,8,n);
 if(p){live_allocations++;g_telemetry_tlsf_live_bytes+=tlsf_block_size(p);}
 return p;
}
int main(int argc,char**argv){
 (void)argv;TX_BYTE_POOL pools[3];
 for(unsigned i=0;i<3;i++){pools[i]=(TX_BYTE_POOL){(unsigned char*)arenas[i],0,sizeof(arenas[i])};telemetry_tlsf_register_pool(&pools[i]);}
 if(argc>1 && strcmp(argv[1],"init-failure")==0){fail_init=1;assert(!telemetry_tlsf_malloc(32));assert(g_telemetry_tlsf_init_failed);assert(!g_telemetry_tlsf_active);assert(mock_irq==0);return 0;}
 void *first=telemetry_tlsf_malloc(0);assert(first&&g_telemetry_tlsf_active);telemetry_tlsf_free(first);
 assert(!telemetry_tlsf_malloc(SIZE_MAX));
 if(argc>1 && strcmp(argv[1],"fragmentation")==0){
  // Reproduce plentiful free bytes with no decode-sized contiguous TLSF block.
  void *keepers[256]={0},*temporary[256]={0};unsigned count=0;
  for(;count<256;count++){
   keepers[count]=ordinary_only(768);
   if(!keepers[count])break;
   memset(keepers[count],0x5a,768);
   temporary[count]=ordinary_only(1024);
   if(!temporary[count]){count++;break;}
  }
  assert(count>20);
  for(unsigned i=0;i<count;i++)telemetry_tlsf_free(temporary[i]);
  void *tails[128];unsigned tail_count=0;
  while(tail_count<128&&(tails[tail_count]=ordinary_only(676)))tail_count++;
  assert(g_rf_large_slot_live==0);
  // The ordinary fragmented heap cannot satisfy even the captured 676-byte request.
  void *ordinary=tlsf_memalign(allocator,8,676);assert(!ordinary);
  // Discovery must use fragmented small blocks plus its large fallback buffer.
  const uint32_t before_bitmap=reserve_used, before_live=g_telemetry_tlsf_live_bytes;
  assert(telemetry_tlsf_admit(8192,2048));
  assert(reserve_used==before_bitmap&&g_telemetry_tlsf_live_bytes==before_live);
  assert(telemetry_tlsf_admit(4096,2048));
  void *decode=telemetry_tlsf_malloc(2048),*nested=telemetry_tlsf_malloc(4096);
  assert(decode&&nested&&decode!=nested&&g_rf_large_slot_live==2);
  memset(decode,0x6b,2048);memset(nested,0x7c,4096);
  void *medium=telemetry_tlsf_malloc(676);assert(medium);memset(medium,0x4d,676);
  assert(!telemetry_tlsf_admit(8192,2048));
  void *guard_block=telemetry_tlsf_malloc(1024);assert(guard_block);
  // A tiny frame estimate must not admit work whose owned object needs 676 B.
  assert(!telemetry_tlsf_admit(384,160));
  telemetry_tlsf_free(guard_block);telemetry_tlsf_free(medium);
  telemetry_tlsf_free(decode);assert(!telemetry_tlsf_admit(8192,2048));
  decode=telemetry_tlsf_malloc(2048);assert(decode);
  for(unsigned j=0;j<4096;j++)assert(((unsigned char*)nested)[j]==0x7c);
  telemetry_tlsf_free(decode);telemetry_tlsf_free(nested);
  void *whole=telemetry_tlsf_malloc(8192);
  assert(whole&&large_slot_index(whole)==0&&reserve_lengths[0]==32);
  memset(whole,0x3e,8192);telemetry_tlsf_free(whole);
  assert(!reserve_used&&!reserve_live_bytes);
  for(unsigned i=0;i<count;i++){
   for(unsigned j=0;j<768;j++)assert(((unsigned char*)keepers[i])[j]==0x5a);
   telemetry_tlsf_free(keepers[i]);
  }
  for(unsigned i=0;i<tail_count;i++)telemetry_tlsf_free(tails[i]);
  assert(!g_rf_large_slot_live&&!g_telemetry_tlsf_live_bytes);
  assert(telemetry_tlsf_admit(8192,8192));return 0;
 }
 const uint32_t initial_free=g_telemetry_tlsf_free_bytes, initial_largest=g_telemetry_tlsf_largest_free;
 const unsigned startup_calls=calls;
 assert(telemetry_tlsf_admit(4096, 2048));
 assert(!telemetry_tlsf_admit(SIZE_MAX, 1));
 assert(!telemetry_tlsf_admit(0, SIZE_MAX));
 assert(mock_irq==0);
 void *items[64]={0};size_t sizes[64]={0};uint32_t rng=42;
 for(unsigned step=0;step<100000;step++){
  rng=rng*1664525U+1013904223U;unsigned i=(rng>>16)%64;
  if(items[i]){
   for(unsigned j=0;j<sizes[i];j++)assert(((unsigned char*)items[i])[j]==i+1);
   telemetry_tlsf_free(items[i]);items[i]=0;
  }else{
   sizes[i]=1+rng%8192;items[i]=telemetry_tlsf_malloc(sizes[i]);
   if(items[i]){assert(((uintptr_t)items[i]&7)==0);memset(items[i],i+1,sizes[i]);}
  }
  assert(mock_irq==0);
  {
   const uint32_t walks=g_telemetry_tlsf_snapshot_count;
   const uint32_t live=g_telemetry_tlsf_live_bytes;
   size_t request=512U+rng%8192U;
   if (telemetry_tlsf_admit(request,request)) {
    void *scratch=telemetry_tlsf_malloc(request);
    assert(scratch);telemetry_tlsf_free(scratch);
   }
   assert(g_telemetry_tlsf_snapshot_count==walks);
   assert(g_telemetry_tlsf_live_bytes==live);
  }
 }
 for(unsigned i=0;i<64;i++)telemetry_tlsf_free(items[i]);
 assert(g_telemetry_tlsf_live_bytes==0&&g_telemetry_tlsf_peak_bytes>0);
 assert(!telemetry_tlsf_malloc(SIZE_MAX));
 assert(g_telemetry_tlsf_free_bytes==initial_free&&g_telemetry_tlsf_largest_free==initial_largest);
 // Exhaust then release every block; freed memory must be reusable.
 void*full[128];unsigned count=0;
 while(count<128&&(full[count]=telemetry_tlsf_malloc(1024)))count++;
 assert(count>0&&count<128);
 assert(!telemetry_tlsf_admit(4096, 2048));
 for(unsigned i=0;i<count;i++)telemetry_tlsf_free(full[i]);
 assert(telemetry_tlsf_admit(4096, 2048));
 mock_irq=1;assert(telemetry_tlsf_admit(4096,2048)&&mock_irq==1);first=telemetry_tlsf_malloc(4112);assert(first&&mock_irq==1);telemetry_tlsf_free(first);assert(mock_irq==1);mock_irq=0;
 assert(calls==startup_calls);telemetry_tlsf_free(NULL);
 assert(g_telemetry_tlsf_live_bytes==0);
}
''')
            exe=str(p/'test')
            subprocess.run(['cc','-std=c11','-g','-fsanitize=address,undefined','-I',tmp,'-I',str(ROOT/'Core/Inc'),'-I',str(ROOT/'Core/Src'),'-I',str(ROOT/'third_party/tlsf'),str(p/'test.c'),str(ROOT/'third_party/tlsf/tlsf.c'),'-o',exe],check=True)
            subprocess.run([exe],check=True)
            subprocess.run([exe,'init-failure'],check=True)
            subprocess.run([exe,'fragmentation'],check=True)
