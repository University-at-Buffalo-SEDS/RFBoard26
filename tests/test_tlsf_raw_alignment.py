"""Raw allocation must remain usable before Rust aligns its owned object."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]
class RawTlsfAlignmentTests(unittest.TestCase):
    def test_small_raw_block_and_rust_layout_alignment(self):
        source = (ROOT / "Core/Src/telemetry_tlsf.c").read_text()
        start = source.index("void *telemetry_tlsf_malloc(size_t size)")
        end = source.index("\nvoid telemetry_tlsf_free", start)
        function = source[start:end]
        harness = r'''#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#define RESERVE_BLOCK_BYTES 256U
static unsigned irq, allocations;
static uintptr_t arena[256];
static void *allocator;
static unsigned live_allocations;
static uint32_t g_telemetry_tlsf_live_bytes, g_telemetry_tlsf_peak_bytes;
static uint32_t g_telemetry_tlsf_failure_request, g_telemetry_tlsf_failures;
static unsigned char *raw = (unsigned char *)arena + 4;
static uint32_t __get_PRIMASK(void){return irq;}
static void __disable_irq(void){irq=1;}
static void __set_PRIMASK(uint32_t mask){irq=mask;}
static int initialize(void){return 1;}
static size_t tlsf_block_size_max(void){return SIZE_MAX;}
static void *tlsf_malloc(void *pool,size_t size){(void)pool;assert(irq&&size);allocations++;return raw;}
static size_t tlsf_block_size(void *ptr){assert(ptr==raw);return 1024;}
static void snapshot(void){}
static __attribute__((unused)) void *large_slot_allocate(size_t size){(void)size;return NULL;}
static __attribute__((unused)) int large_slot_index(void *ptr){(void)ptr;return -1;}
static __attribute__((unused)) unsigned reserve_lengths[32];
'''
        main = r'''
int main(void){
 // Reproduce a naturally aligned small free block, not aligned to eight bytes.
 assert((uintptr_t)raw % 8 == 4);
 assert(telemetry_tlsf_malloc(10)==raw && allocations==1 && irq==0);
 // Mirror Rust's header/alignment contract without requiring the raw block to
 // have the final layout alignment. Check header ownership and payload bounds.
 for(size_t align=1;align<=256;align*=2){
  for(size_t size=1;size<=128;size+=17){
   size_t effective=align<sizeof(uintptr_t)?sizeof(uintptr_t):align;
   size_t total=size+effective+sizeof(uintptr_t);
   unsigned char *base=telemetry_tlsf_malloc(total);
   uintptr_t address=((uintptr_t)base+sizeof(uintptr_t)+effective-1)&~(effective-1);
   unsigned char *owned=(unsigned char*)address;
   assert(address%effective==0 && owned+size<=base+total);
   ((uintptr_t*)owned)[-1]=(uintptr_t)base;
   memset(owned,0xa5,size);
   assert(((uintptr_t*)owned)[-1]==(uintptr_t)raw);
  }
 }
 assert(!g_telemetry_tlsf_failures && !irq);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            exe = Path(tmp)/"raw-alignment"
            subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror",
                "-fsanitize=address,undefined","-x","c","-","-o",str(exe)],
                input=harness+function+main,text=True,check=True)
            subprocess.run([str(exe)],check=True)
if __name__ == "__main__":
    unittest.main()
