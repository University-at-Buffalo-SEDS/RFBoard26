#ifndef BOARD_UPTIME_H
#define BOARD_UPTIME_H
#include <stdint.h>
/* HAL_GetTick is expressed in milliseconds, independently of IRQ frequency.
 * This is the nominal counter scale, not a claim of oscillator accuracy. */
#define BOARD_UPTIME_FREQUENCY_HZ 1000U
typedef struct { uint64_t ticks; } board_uptime_state;
static inline uint64_t board_uptime_extend(board_uptime_state *state, uint32_t raw)
{
    /* The low word already is the previous raw counter; no duplicate state. */
    state->ticks += (uint32_t)(raw - (uint32_t)state->ticks);
    return state->ticks;
}
static inline uint64_t board_uptime_project(uint64_t now, uint32_t captured)
{
    const uint32_t age = (uint32_t)now - captured;
    return now >= age ? now - age : 0U;
}
/* Sample at least once per 32-bit wrap. No network-clock lock is acquired. */
uint64_t board_uptime_ticks(void);
#endif
