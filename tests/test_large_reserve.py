import pathlib
import subprocess
import tempfile
import unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
class LargeReserveTests(unittest.TestCase):
    def test_fragmentation_is_bounded_and_buffers_coalesce(self):
        code=r"""
#include <assert.h>
#include <stdint.h>
#include <string.h>
static uint32_t mask;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t m){mask=m;}
#include "telemetry_large_reserve.h"
static uint64_t arena[TELEMETRY_RESERVE_BYTES/8];
int main(void){
 assert(!large_reserve_allocate(3640));large_reserve_init(arena);
 assert(!large_reserve_allocate(32) && !large_reserve_allocate(16385));
 // Reproduce concurrent schema scratch sizes observed on the gateway.
 for(unsigned cycle=0;cycle<100000;cycle++){
  void*a=large_reserve_allocate(4112),*b=large_reserve_allocate(3640),*c=large_reserve_allocate(3640);
  assert(a&&b&&c&&a!=b&&b!=c&&a!=c);void *e=large_reserve_allocate(4112);assert(e);assert(!large_reserve_allocate(1024));large_reserve_release(e);
  memset(a,0xa5,4112);memset(b,0xb6,3640);memset(c,0xc7,3640);
  for(unsigned i=0;i<4112;i++)assert(((unsigned char*)a)[i]==0xa5);
  assert(large_reserve_owns(a)&&large_reserve_available()==4352);
  large_reserve_release(b);large_reserve_release(c);
  void*d=large_reserve_allocate(8192);assert(d);large_reserve_release(d);large_reserve_release(a);
  mask=1;d=large_reserve_allocate(16384);assert(d&&mask==1);large_reserve_release(d);assert(mask==1);mask=0;
  assert(large_reserve_available()==16384);
 }
 // Mixed lifetimes must not overlap or retain memory after everything frees.
 void *blocks[32]={0};unsigned sizes[32]={0};uint32_t rng=12345;
 for(unsigned k=0;k<100000;k++) {
  rng=rng*1664525U+1013904223U;unsigned j=(rng>>16)%32;
  if(blocks[j]) {
   for(unsigned x=0;x<sizes[j];x++)assert(((unsigned char*)blocks[j])[x]==j+1);
   large_reserve_release(blocks[j]);blocks[j]=0;
  } else {
   unsigned size=1024+(rng%7169);blocks[j]=large_reserve_allocate(size);
   if(blocks[j]){sizes[j]=size;memset(blocks[j],j+1,size);}
  }
 }
 for(unsigned j=0;j<32;j++)large_reserve_release(blocks[j]);
 assert(large_reserve_available()==16384);
 large_reserve_release(NULL);large_reserve_release((char*)arena+1);
 assert(large_reserve_available()==16384);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            exe=str(pathlib.Path(tmp)/"reserve")
            subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror","-fsanitize=address,undefined","-I",str(ROOT/"Core/Inc"),"-x","c","-","-o",exe],input=code,text=True,check=True)
            subprocess.run([exe],check=True)
