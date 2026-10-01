#ifndef TELEMETRY_TLSF_H
#define TELEMETRY_TLSF_H
#include "tx_api.h"
#include <stddef.h>
/* Register startup pools before the first Rust allocation. TLSF then owns
 * their remaining contiguous storage; ThreadX keeps thread-stack ownership. */
void telemetry_tlsf_register_pool(TX_BYTE_POOL *pool);
void *telemetry_tlsf_malloc(size_t size);
void telemetry_tlsf_free(void *ptr);
void telemetry_tlsf_sample(void);
#endif
