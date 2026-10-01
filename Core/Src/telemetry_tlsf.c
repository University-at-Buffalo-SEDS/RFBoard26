#include "main.h"
#include "telemetry_tlsf.h"
#include "tlsf.h"
#include <stdint.h>

static TX_BYTE_POOL *backing[3];
static pool_t regions[3];
static unsigned backing_count, region_count, init_attempted;
static tlsf_t allocator;
volatile uint32_t g_telemetry_tlsf_active;
volatile uint32_t g_telemetry_tlsf_init_failed;
volatile uint32_t g_telemetry_tlsf_region_bytes;
volatile uint32_t g_telemetry_tlsf_control_bytes;
volatile uint32_t g_telemetry_tlsf_live_bytes;
volatile uint32_t g_telemetry_tlsf_peak_bytes;
volatile uint32_t g_telemetry_tlsf_failure_request;
volatile uint32_t g_telemetry_tlsf_failures;
/* Exact free-space snapshot at startup and failed allocations, not an O(n)
 * heap walk on every packet. Live bytes count occupied TLSF payload blocks. */
volatile uint32_t g_telemetry_tlsf_free_bytes;
volatile uint32_t g_telemetry_tlsf_largest_free;
volatile uint32_t g_telemetry_tlsf_free_blocks;

void telemetry_tlsf_register_pool(TX_BYTE_POOL *pool)
{
    if (!pool || init_attempted || backing_count == 3U) {
        Error_Handler();
        return;
    }
    for (unsigned i = 0; i < backing_count; ++i) {
        if (backing[i] == pool) return;
    }
    backing[backing_count++] = pool;
}
static void snapshot_block(void *ptr, size_t size, int used, void *context)
{
    (void)ptr; (void)context;
    if (!used) {
        g_telemetry_tlsf_free_bytes += size;
        ++g_telemetry_tlsf_free_blocks;
        if (size > g_telemetry_tlsf_largest_free) g_telemetry_tlsf_largest_free = size;
    }
}
static void snapshot(void)
{
    g_telemetry_tlsf_free_bytes = 0;
    g_telemetry_tlsf_largest_free = 0;
    g_telemetry_tlsf_free_blocks = 0;
    for (unsigned i = 0; i < region_count; ++i)
        tlsf_walk_pool(regions[i], snapshot_block, NULL);
}
static int initialize(void)
{
    if (init_attempted) return g_telemetry_tlsf_active != 0;
    init_attempted = 1;
    if (!backing_count) goto fail;
    void *control = NULL;
    const size_t alignment = tlsf_align_size();
    const size_t control_bytes = tlsf_size() + alignment - 1U;
    if (tx_byte_allocate(backing[0], &control, control_bytes, TX_NO_WAIT) != TX_SUCCESS)
        goto fail;
    allocator = tlsf_create((void *)(((uintptr_t)control + alignment - 1U) & ~(alignment - 1U)));
    if (!allocator) goto fail;
    g_telemetry_tlsf_control_bytes = control_bytes;
    for (unsigned i = 0; i < backing_count; ++i) {
        ULONG available = 0;
        if (tx_byte_pool_info_get(backing[i], TX_NULL, &available, TX_NULL,
                                 TX_NULL, TX_NULL, TX_NULL) != TX_SUCCESS) goto fail;
        /* Startup leaves one contiguous free tail in each registered pool.
         * Keep room for ThreadX's allocation header and alignment. A failed
         * takeover is fatal to this experiment; never mix pointer ownership. */
        if (available <= 4U * sizeof(void *) + tlsf_pool_overhead() + tlsf_block_size_min())
            goto fail;
        const size_t bytes = (available - 4U * sizeof(void *)) & ~(alignment - 1U);
        void *memory = NULL;
        if (tx_byte_allocate(backing[i], &memory, bytes, TX_NO_WAIT) != TX_SUCCESS) goto fail;
        pool_t region = tlsf_add_pool(allocator, memory, bytes);
        if (!region) goto fail;
        regions[region_count++] = region;
        g_telemetry_tlsf_region_bytes += bytes;
    }
    g_telemetry_tlsf_active = 1;
    snapshot();
    return 1;
fail:
    g_telemetry_tlsf_init_failed = 1;
    return 0;
}
void *telemetry_tlsf_malloc(size_t size)
{
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    void *ptr = NULL;
    if (initialize() && size <= tlsf_block_size_max() - 64U)
        ptr = tlsf_memalign(allocator, 8U, size ? size : 1U);
    if (ptr) {
        g_telemetry_tlsf_live_bytes += tlsf_block_size(ptr);
        if (g_telemetry_tlsf_live_bytes > g_telemetry_tlsf_peak_bytes)
            g_telemetry_tlsf_peak_bytes = g_telemetry_tlsf_live_bytes;
    } else {
        g_telemetry_tlsf_failure_request = size;
        ++g_telemetry_tlsf_failures;
        snapshot();
    }
    __set_PRIMASK(saved);
    return ptr;
}
void telemetry_tlsf_free(void *ptr)
{
    if (!ptr) return;
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    g_telemetry_tlsf_live_bytes -= tlsf_block_size(ptr);
    tlsf_free(allocator, ptr);
    __set_PRIMASK(saved);
}

void telemetry_tlsf_sample(void)
{
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    snapshot();
    __set_PRIMASK(saved);
}
