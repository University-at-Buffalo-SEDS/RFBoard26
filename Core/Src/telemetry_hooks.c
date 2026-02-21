// Core/Src/telemetry_alloc.c
#include "tx_api.h"
#include <stddef.h>
#include <stdio.h>

/*
 * Rust expects these functions to exist for heap allocations:
 *
 *   void *telemetryMalloc(size_t);
 *   void telemetryFree(void *);
 *   void seds_error_msg(const char *str, size_t len);
 *
 */

/* Default internal heap used only if an external pool is not provided. */
#define RUST_HEAP_SIZE  (32 * 1024u)  // this will need to be tuned
static TX_BYTE_POOL rust_byte_pool_internal;
static UCHAR rust_heap_internal[RUST_HEAP_SIZE];

/* Pointer to an externally-provided ThreadX byte pool. If non-NULL,
   telemetryMalloc/Free will use that pool. */
static TX_BYTE_POOL *rust_byte_pool_external = NULL;

/* Register an external `TX_BYTE_POOL` for Rust allocations. Call this
   from `App_ThreadX_Init` (or similar) passing the application's
   `TX_BYTE_POOL *` so Rust will use the existing pool. */
void telemetry_set_byte_pool(TX_BYTE_POOL *pool)
{
    rust_byte_pool_external = pool;
}

void rust_heap_init(void)
{
    static UINT initialized = 0;
    if (initialized || rust_byte_pool_external) {
        return;
    }

    UINT status = tx_byte_pool_create(&rust_byte_pool_internal,
                                      "rust_heap",
                                      rust_heap_internal,
                                      sizeof(rust_heap_internal));
    if (status != TX_SUCCESS) {
        /* If this fails, you're in deep trouble – spin or assert */
        while (1) { }
    }

    initialized = 1;
}

void *telemetryMalloc(size_t xSize)
{
    void *ptr = NULL;

    /* Make sure pool is ready – safe to call multiple times */
    rust_heap_init();

    /* Prefer external pool if provided, otherwise use internal one. */
    TX_BYTE_POOL *pool = rust_byte_pool_external ? rust_byte_pool_external : &rust_byte_pool_internal;

    /* TX_NO_WAIT: allocator is fast and non-blocking */
    UINT status = tx_byte_allocate(pool, &ptr, xSize, TX_NO_WAIT);
    if (status != TX_SUCCESS) {
        return NULL;
    }
    return ptr;
}

void telemetryFree(void *pv)
{
    if (pv == NULL) {
        return;
    }

    /* tx_byte_release doesn't require the pool pointer here; it will
       release the memory previously allocated by tx_byte_allocate. */
    (void)tx_byte_release(pv);
}

void seds_error_msg(const char *str, size_t len)
{
    (void)len;
    printf("%s\n", str);
}