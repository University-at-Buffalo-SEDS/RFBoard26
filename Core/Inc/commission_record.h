#ifndef COMMISSION_RECORD_H
#define COMMISSION_RECORD_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* Application NVM only. New five-domain v2 retains the complete original
 * first-leg generation; AB/VB v1 and every network byte remain unchanged. */
#define COMMISSION_RECORD_KEY UINT32_C(0x42525043)
#define COMMISSION_RECORD_SIZE 40U
typedef struct {uint8_t state,kind;uint32_t epoch;uint64_t source,nonce,issuer,generation;} CommissionRecord;
static inline uint64_t commission_read64(const uint8_t*p){uint64_t n=0;for(unsigned i=0;i<8;i++)n|=(uint64_t)p[i]<<(8*i);return n;}
static inline void commission_write64(uint8_t*p,uint64_t n){for(unsigned i=0;i<8;i++)p[i]=(uint8_t)(n>>(8*i));}
static inline uint32_t commission_read32(const uint8_t*p){return(uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static inline void commission_write32(uint8_t*p,uint32_t n){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(n>>(8*i));}
static inline bool commission_record_decode(const uint8_t*p,size_t size,CommissionRecord*out){
 if(size!=40U||p[0]!=2U||p[1]>1U||p[2]||p[3]>1U)return false;
 CommissionRecord r={.state=p[1],.kind=p[3],.epoch=commission_read32(p+4),.source=commission_read64(p+8),.nonce=commission_read64(p+16),.issuer=commission_read64(p+24),.generation=commission_read64(p+32)};
 if(!r.epoch||r.epoch==UINT32_MAX||!r.source||r.source==UINT64_MAX||!r.nonce||!r.issuer||!r.generation)return false;
 *out=r;return true;
}
static inline void commission_record_encode(uint8_t*p,const CommissionRecord*r){p[0]=2;p[1]=r->state;p[2]=0;p[3]=r->kind;commission_write32(p+4,r->epoch);commission_write64(p+8,r->source);commission_write64(p+16,r->nonce);commission_write64(p+24,r->issuer);commission_write64(p+32,r->generation);}
#endif
