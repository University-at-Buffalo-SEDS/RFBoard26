#ifndef NETWORK_STAMP_H
#define NETWORK_STAMP_H
#include <stdint.h>
/* A SEDSnet 6 network stamp (time_epoch, timestamp ms) as one ordered 64-bit
 * key: the epoch in the top 16 bits, the stamp in the low 48 (Unix ms reach
 * 2^48 in the year 10889). Keys order exactly like seds_stamp_cmp, so stored
 * marks (recovery floor, command cutoff, last applied stamp) keep their
 * 8-byte fields; a mark stored before SEDSnet 6 reads as epoch 0. 0 is
 * "none": a zero or out-of-range stamp has no key. */
#define NETWORK_STAMP_MS_MASK ((UINT64_C(1) << 48) - 1U)
static inline uint64_t network_stamp_key(uint16_t epoch, uint64_t ms)
{
 return ms == 0U || ms > NETWORK_STAMP_MS_MASK ? 0U : ((uint64_t)epoch << 48) | ms;
}
static inline uint16_t network_stamp_epoch(uint64_t key) { return (uint16_t)(key >> 48); }
static inline uint64_t network_stamp_ms(uint64_t key) { return key & NETWORK_STAMP_MS_MASK; }
/* The key of a packet view handed out by the SDK. */
#define NETWORK_STAMP_OF(view) network_stamp_key((view)->time_epoch, (view)->timestamp)
#endif
