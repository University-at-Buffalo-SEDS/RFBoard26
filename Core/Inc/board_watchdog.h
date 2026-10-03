#ifndef BOARD_WATCHDOG_H
#define BOARD_WATCHDOG_H
#include <stdint.h>
#define BOARD_WATCHDOG_REQUIRED_MASK (BOARD_WATCHDOG_ACQUISITION | BOARD_WATCHDOG_NETWORK)

#define BOARD_WATCHDOG_NETWORK (1UL << 0)
#define BOARD_WATCHDOG_CONTROL (1UL << 1)
#define BOARD_WATCHDOG_SAFETY  (1UL << 2)
#define BOARD_WATCHDOG_ACQUISITION (1UL << 3)
#define BOARD_WATCHDOG_STORAGE (1UL << 4)

extern volatile uint32_t g_watchdog_reset_flags;
extern volatile uint32_t g_watchdog_started;
extern volatile uint32_t g_watchdog_feed_count;
extern volatile uint32_t g_watchdog_seen_mask;
extern volatile uint32_t g_watchdog_missing_mask;
extern volatile uint32_t g_watchdog_config_error;
void board_watchdog_start(void);
void board_watchdog_progress(uint32_t task);
#endif
