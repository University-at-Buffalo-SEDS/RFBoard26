#ifndef TELEMETRY_LARGE_RESERVE_H
#define TELEMETRY_LARGE_RESERVE_H
#include <stddef.h>
#include <stdint.h>

/* Adjacent 256-byte units coalesce immediately on free. Allocation inspects
 * at most 64 units, never waits, and cannot create byte-pool metadata holes. */
#define TELEMETRY_RESERVE_BYTES 16384U
#define TELEMETRY_RESERVE_UNIT 256U
#ifndef TELEMETRY_RESERVE_MIN
#define TELEMETRY_RESERVE_MIN 1024U
#endif
static uint8_t *large_reserve_base;
static uint8_t large_reserve_units[64];

static void large_reserve_init(void *base)
{
    large_reserve_base = base;
    for (unsigned i = 0U; i < 64U; ++i) large_reserve_units[i] = 0U;
}
static void *large_reserve_allocate(size_t size)
{
    if (!large_reserve_base || size < TELEMETRY_RESERVE_MIN ||
        size > TELEMETRY_RESERVE_BYTES) return NULL;
    const unsigned needed = (size + TELEMETRY_RESERVE_UNIT - 1U) / TELEMETRY_RESERVE_UNIT;
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    unsigned run = 0U;
    /* Put large scratch at the high end; retained small records grow from
     * the low end, avoiding a small tail hole between serialization buffers. */
    for (unsigned step = 0U; step < 64U; ++step) {
        const unsigned i = size >= 3072U ? 63U - step : step;
        run = large_reserve_units[i] == 0U ? run + 1U : 0U;
        if (run == needed) {
            const unsigned start = size >= 3072U ? i : i + 1U - needed;
            large_reserve_units[start] = needed;
            for (unsigned j = start + 1U; j < start + needed; ++j) large_reserve_units[j] = 255U;
            __set_PRIMASK(saved);
            return large_reserve_base + start * TELEMETRY_RESERVE_UNIT;
        }
    }
    __set_PRIMASK(saved);
    return NULL;
}
static int large_reserve_owns(const void *ptr)
{
    const uintptr_t p = (uintptr_t)ptr, base = (uintptr_t)large_reserve_base;
    return large_reserve_base && p >= base && p - base < TELEMETRY_RESERVE_BYTES;
}
static void large_reserve_release(void *ptr)
{
    if (!large_reserve_owns(ptr)) return;
    const uintptr_t offset = (uintptr_t)ptr - (uintptr_t)large_reserve_base;
    if (offset % TELEMETRY_RESERVE_UNIT != 0U) return;
    const unsigned i = offset / TELEMETRY_RESERVE_UNIT;
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    const unsigned units = large_reserve_units[i];
    if (units != 0U && units != 255U && units <= 64U - i) {
        for (unsigned j = i; j < i + units; ++j) large_reserve_units[j] = 0U;
    }
    __set_PRIMASK(saved);
}
static uint32_t large_reserve_available(void)
{
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    unsigned free_units = 0U;
    if (large_reserve_base) {
        for (unsigned i = 0U; i < 64U; ++i) free_units += large_reserve_units[i] == 0U;
    }
    __set_PRIMASK(saved);
    return free_units * TELEMETRY_RESERVE_UNIT;
}
#endif
