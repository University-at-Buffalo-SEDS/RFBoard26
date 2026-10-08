#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include "tlsf.h"
static uint64_t arena[8192];
static uint64_t hash;
static void digest(void *p,size_t n,int used,void *arg) {
 (void)arg; hash=hash*131U+(uintptr_t)p+n+(unsigned)used;
}
static uint64_t snapshot(tlsf_t t) { hash=0;tlsf_walk_pool(tlsf_get_pool(t),digest,0);return hash; }
int main(void) {
 tlsf_t t=tlsf_create_with_pool(arena,sizeof(arena));assert(t);
 assert(!tlsf_can_memalign(t,0,8));assert(!tlsf_can_memalign(t,3,8));
 assert(!tlsf_can_memalign(t,8,SIZE_MAX));assert(!tlsf_can_memalign(t,SIZE_MAX,8));
 void *slots[64]={0};uint32_t rng=1;
 for(unsigned i=0;i<6000;i++) {
  rng=rng*1664525U+1013904223U;unsigned slot=(rng>>16)%64;
  if(slots[slot]){tlsf_free(t,slots[slot]);slots[slot]=0;}
  else slots[slot]=tlsf_memalign(t,8,1+rng%4096);
  size_t align=(size_t)1<<(3+(rng%6)),n=(rng>>8)%8192;
  uint64_t before=snapshot(t);
  int predicted=tlsf_can_memalign(t,align,n);
  assert(snapshot(t)==before && tlsf_check(t)==0);
  void *p=tlsf_memalign(t,align,n);
  assert(predicted==(p!=0));
  if(p) assert(((uintptr_t)p&(align-1))==0);
  tlsf_free(t,p);assert(tlsf_check(t)==0);
 }
 for(unsigned i=0;i<64;i++)tlsf_free(t,slots[i]);
 assert(tlsf_can_memalign(t,8,32768));
}
