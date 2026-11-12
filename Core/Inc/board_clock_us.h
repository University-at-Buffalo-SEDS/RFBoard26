#ifndef BOARD_CLOCK_US_H
#define BOARD_CLOCK_US_H
/* Microseconds since boot for time sync stamps: the ThreadX 1 ms tick plus
 * the SysTick count within the current tick. SysTick runs from the core clock
 * in Sleep (WFI) too, unlike DWT CYCCNT, which stops while the core is
 * gated; the 32-bit tick wraps after 49.7 days like the board's ms clock.
 * Callable from interrupts and threads. */
#include <stdint.h>
#include "main.h"
#include "tx_api.h"

static inline uint64_t board_now_us(void) {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t ticks = (uint32_t)tx_time_get();
  uint32_t val = SysTick->VAL;
  /* The counter reloaded but its tick interrupt has not run yet. */
  if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0U) {
    val = SysTick->VAL;
    ++ticks;
  }
  __set_PRIMASK(primask);
  const uint32_t load = SysTick->LOAD + 1U;
  return (uint64_t)ticks * (1000000ULL / TX_TIMER_TICKS_PER_SECOND) +
         ((uint64_t)(load - 1U - val) * (1000000ULL / TX_TIMER_TICKS_PER_SECOND)) / load;
}

#endif
