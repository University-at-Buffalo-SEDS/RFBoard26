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
static uint32_t live_allocations;
/* Keep decode-sized blocks out of the small-packet allocation stream. These
 * slots are carved from the existing pool, not additional static RAM. */
#define LARGE_SLOT_BYTES 4096U
#define LARGE_SLOT_COUNT 2U
#define RESERVE_BLOCK_BYTES 256U
#define RESERVE_BLOCK_COUNT 32U
static uint8_t *large_slots;
static uint32_t reserve_used;
static uint8_t reserve_lengths[RESERVE_BLOCK_COUNT];
static uint32_t reserve_live_bytes;
volatile uint32_t g_rf_large_slot_live;
volatile uint32_t g_rf_large_slot_peak;
volatile uint32_t g_rf_admission_last_additional;
volatile uint32_t g_rf_admission_last_largest;

/* A fixed bitmap avoids heap headers and bounds each search to 32 blocks.
 * Only the allocation start records its length; pointers never move. */
static int reserve_find(size_t size)
{
    if (!large_slots || size == 0U || size > RESERVE_BLOCK_BYTES * RESERVE_BLOCK_COUNT) return -1;
    const unsigned blocks = (size + RESERVE_BLOCK_BYTES - 1U) / RESERVE_BLOCK_BYTES;
    const uint32_t bits = UINT32_MAX >> (RESERVE_BLOCK_COUNT - blocks);
    for (unsigned i = 0; i + blocks <= RESERVE_BLOCK_COUNT; ++i)
        if ((reserve_used & (bits << i)) == 0U) return (int)i;
    return -1;
}
static void *large_slot_allocate(size_t size)
{
    const int index = reserve_find(size);
    if (index < 0) return NULL;
    const unsigned blocks = (size + RESERVE_BLOCK_BYTES - 1U) / RESERVE_BLOCK_BYTES;
    reserve_used |= (UINT32_MAX >> (RESERVE_BLOCK_COUNT - blocks)) << (unsigned)index;
    reserve_lengths[index] = blocks;
    reserve_live_bytes += blocks * RESERVE_BLOCK_BYTES;
    ++g_rf_large_slot_live;
    if (g_rf_large_slot_live > g_rf_large_slot_peak)
        g_rf_large_slot_peak = g_rf_large_slot_live;
    return large_slots + (unsigned)index * RESERVE_BLOCK_BYTES;
}
static int large_slot_index(void *ptr)
{
    const uintptr_t address = (uintptr_t)ptr, start = (uintptr_t)large_slots;
    if (!large_slots || address < start || address >= start + LARGE_SLOT_COUNT * LARGE_SLOT_BYTES)
        return -1;
    return (address - start) / RESERVE_BLOCK_BYTES;
}
volatile uint32_t g_rf_memory_admission_drops;
volatile uint32_t g_telemetry_tlsf_peak_bytes;
volatile uint32_t g_telemetry_tlsf_failure_request;
volatile uint32_t g_telemetry_tlsf_failures;
/* Exact free-space snapshot at startup and failed allocations, not an O(n)
 * heap walk on every packet. Live bytes count occupied TLSF payload blocks. */
volatile uint32_t g_telemetry_tlsf_free_bytes;
volatile uint32_t g_telemetry_tlsf_largest_free;
volatile uint32_t g_telemetry_tlsf_free_blocks;
volatile uint32_t g_telemetry_tlsf_snapshot_count;

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
    ++g_telemetry_tlsf_snapshot_count;
    g_telemetry_tlsf_free_bytes = 0;
    g_telemetry_tlsf_largest_free = 0;
    g_telemetry_tlsf_free_blocks = 0;
    for (unsigned i = 0; i < region_count; ++i)
        tlsf_walk_pool(regions[i], snapshot_block, NULL);
    if (large_slots) {
        g_telemetry_tlsf_free_bytes += LARGE_SLOT_COUNT * LARGE_SLOT_BYTES - reserve_live_bytes;
        unsigned run = 0;
        for (unsigned i = 0; i < RESERVE_BLOCK_COUNT; ++i) {
            if (reserve_used & (1UL << i)) run = 0;
            else {
                if (run == 0U) ++g_telemetry_tlsf_free_blocks;
                ++run;
                const uint32_t bytes = run * RESERVE_BLOCK_BYTES;
                if (bytes > g_telemetry_tlsf_largest_free) g_telemetry_tlsf_largest_free = bytes;
            }
        }
    }
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
    large_slots = tlsf_memalign(allocator, 8U, LARGE_SLOT_COUNT * LARGE_SLOT_BYTES);
    if (!large_slots) goto fail;
    /* Reserve ownership overhead/padding conservatively in admission accounting. */
    g_telemetry_tlsf_region_bytes -= tlsf_block_size(large_slots) -
        LARGE_SLOT_COUNT * LARGE_SLOT_BYTES + 2U * sizeof(void *);
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
    if (initialize() && size <= tlsf_block_size_max() - 64U) {
        /* Startup topology/replay buffers should not occupy the reserve. */
        ptr = tlsf_memalign(allocator, 8U, size ? size : 1U);
        if (!ptr) ptr = large_slot_allocate(size);
    }
    if (ptr) {
        ++live_allocations;
        const int index = large_slot_index(ptr);
        g_telemetry_tlsf_live_bytes += index >= 0 ?
            reserve_lengths[index] * RESERVE_BLOCK_BYTES : tlsf_block_size(ptr);
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
    --live_allocations;
    const int slot = large_slot_index(ptr);
    if (slot >= 0) {
        const unsigned blocks = reserve_lengths[slot];
        reserve_used &= ~((UINT32_MAX >> (RESERVE_BLOCK_COUNT - blocks)) << (unsigned)slot);
        reserve_lengths[slot] = 0U;
        --g_rf_large_slot_live;
        reserve_live_bytes -= blocks * RESERVE_BLOCK_BYTES;
        g_telemetry_tlsf_live_bytes -= blocks * RESERVE_BLOCK_BYTES;
    } else {
        g_telemetry_tlsf_live_bytes -= tlsf_block_size(ptr);
        tlsf_free(allocator, ptr);
    }
    __set_PRIMASK(saved);
}

void telemetry_tlsf_sample(void)
{
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    snapshot();
    __set_PRIMASK(saved);
}

/* Keep admission bounded even with a fragmented heap. The accounting below
 * conservatively includes metadata for allocated and free blocks. Checking an
 * actual aligned scratch allocation uses TLSF's bitmap lookup and bounded
 * split/coalesce operations; walking every heap block here delayed CAN RX.
 * The probe is released before returning and cannot recurse into this hook. */
bool telemetry_tlsf_admit(size_t additional, size_t largest)
{
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    bool allowed = initialize() != 0;
    const size_t reserve = additional <= 512U ? 512U : 4096U;
    /* Even a small wire ACK can allocate a larger owned queue/decoder object:
     * hardware captured a 676-byte request after a 160-byte scratch estimate.
     * Preserve the smaller ACK reserve, but admit its real working block. */
    if (additional < 1024U) additional = 1024U;
    if (largest != 0U && largest < 1024U) largest = 1024U;
    const size_t occupied = (size_t)g_telemetry_tlsf_live_bytes +
        ((size_t)live_allocations + region_count) * 2U * sizeof(void *);
    const size_t available = g_telemetry_tlsf_region_bytes > occupied ?
        g_telemetry_tlsf_region_bytes - occupied : 0U;
    allowed = allowed && additional <= available && reserve <= available - additional;
    if (allowed && largest != 0U) {
        if (largest > tlsf_block_size_max() - 64U) {
            allowed = false;
        } else {
            /* A discovery snapshot clones many small strings, not just its
             * largest block. If TLSF cannot supply scratch, the fallback
             * must cover its largest buffer and control reserve; ordinary
             * fragmented blocks must cover the remaining operation budget. */
            const size_t reserve_free = RESERVE_BLOCK_BYTES * RESERVE_BLOCK_COUNT - reserve_live_bytes;
            const bool slot_available = reserve_find(largest) >= 0 &&
                largest <= reserve_free && reserve <= reserve_free - largest;
            const size_t padding = slot_available ?
                ((largest + RESERVE_BLOCK_BYTES - 1U) / RESERVE_BLOCK_BYTES) * RESERVE_BLOCK_BYTES - largest : 0U;
            void *scratch = tlsf_memalign(allocator, 8U, largest);
            if (scratch) {
                tlsf_free(allocator, scratch);
                allowed = true;
            } else {
                allowed = slot_available && padding <= available - additional - reserve;
                /* Fragmented ordinary blocks can still hold the small strings
                 * in a discovery snapshot. Hold their full budget while
                 * testing, then release it, rather than rejecting discovery
                 * forever just because its largest buffer needs fallback. */
                void *head = NULL;
                size_t remaining = additional > largest ? additional - largest : 0U;
                unsigned probes = 0U;
                while (allowed && remaining != 0U && probes++ < 128U) {
                    const size_t chunk = remaining > 128U ? 128U : remaining;
                    void *block = tlsf_memalign(allocator, 8U,
                        chunk < sizeof(void *) ? sizeof(void *) : chunk);
                    if (!block) { allowed = false; break; }
                    *(void **)block = head;
                    head = block;
                    remaining -= chunk;
                }
                if (remaining != 0U) allowed = false;
                while (head) {
                    void *next = *(void **)head;
                    tlsf_free(allocator, head);
                    head = next;
                }
            }
        }
    }
    if (!allowed) {
        ++g_rf_memory_admission_drops;
        g_rf_admission_last_additional = additional;
        g_rf_admission_last_largest = largest;
    }
    __set_PRIMASK(saved);
    return allowed;
}
