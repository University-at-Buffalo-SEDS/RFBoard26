#include "board_watchdog.h"
#include "main.h"
#include "tx_api.h"

#ifndef BOARD_WATCHDOG_ENABLE
#define BOARD_WATCHDOG_ENABLE 0
#endif
#ifndef TELEMETRY_ENABLED
#define WATCHDOG_REQUIRED (BOARD_WATCHDOG_REQUIRED_MASK & ~BOARD_WATCHDOG_NETWORK)
#else
#define WATCHDOG_REQUIRED BOARD_WATCHDOG_REQUIRED_MASK
#endif
#ifndef BOARD_WATCHDOG_REQUIRED_MASK
#define BOARD_WATCHDOG_REQUIRED_MASK BOARD_WATCHDOG_NETWORK
#endif

volatile uint32_t g_watchdog_reset_flags;
volatile uint32_t g_watchdog_started;
volatile uint32_t g_watchdog_feed_count;
volatile uint32_t g_watchdog_seen_mask;
volatile uint32_t g_watchdog_missing_mask = WATCHDOG_REQUIRED;
volatile uint32_t g_watchdog_config_error;
#if BOARD_WATCHDOG_ENABLE
static ULONG last_feed_tick;
#endif

void board_watchdog_start(void)
{
#if defined(STM32H533xx) || defined(STM32H523xx)
    g_watchdog_reset_flags = RCC->RSR;
#else
    g_watchdog_reset_flags = RCC->CSR;
#endif
    __HAL_RCC_CLEAR_RESET_FLAGS();
#if BOARD_WATCHDOG_ENABLE
    /* Independent LSI oscillator: nominal 16.384 s at 32 kHz. This is a
     * last-resort reset deadline, not an actuator pulse timeout. */
    IWDG->KR = 0xCCCCU;
    IWDG->KR = 0x5555U;
    IWDG->PR = 6U; /* /256 */
    IWDG->RLR = 2047U;
    /* H5 ONF stays set while enabled; EWIF is an event, not a busy bit.
     * Only configuration-update bits must clear. Prescaled LSI synchronization
     * can take tens of milliseconds, so a CPU-dependent spin budget is unsafe. */
    uint32_t updates = IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU;
#ifdef IWDG_SR_EWU
    updates |= IWDG_SR_EWU;
#endif
    const uint32_t started = HAL_GetTick();
    g_watchdog_started = 0U;
    g_watchdog_config_error = 0U;
    while ((IWDG->SR & updates) != 0U) {
        if ((uint32_t)(HAL_GetTick() - started) >= 100U) {
            g_watchdog_config_error = 1U;
            return;
        }
        __NOP();
    }
    IWDG->KR = 0xAAAAU;
    g_watchdog_started = 1U;
#endif
}

void board_watchdog_progress(uint32_t task)
{
#if BOARD_WATCHDOG_ENABLE
    if (g_watchdog_started == 0U) return;
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    g_watchdog_seen_mask |= task & WATCHDOG_REQUIRED;
    g_watchdog_missing_mask = WATCHDOG_REQUIRED & ~g_watchdog_seen_mask;
    const ULONG now = tx_time_get();
    if (g_watchdog_missing_mask == 0U &&
        (ULONG)(now - last_feed_tick) >= TX_TIMER_TICKS_PER_SECOND / 4U) {
        /* Each required task must check in again after every feed. A live
         * scheduler or network interrupt alone cannot keep a stuck board alive. */
        IWDG->KR = 0xAAAAU;
        g_watchdog_feed_count++;
        g_watchdog_seen_mask = 0U;
        g_watchdog_missing_mask = WATCHDOG_REQUIRED;
        last_feed_tick = now;
    }
    __set_PRIMASK(saved);
#else
    (void)task;
#endif
}
