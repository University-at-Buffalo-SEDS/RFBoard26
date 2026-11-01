#ifndef BOARD_RECOVERY_BOOT_H
#define BOARD_RECOVERY_BOOT_H
#include <stdint.h>
/* A verified monotonic persisted boot generation; zero disables recovery.
 * It is not an authentication secret or a physical board identity. */
uint64_t board_recovery_boot_generation(void);
/* RECOVER cannot initialize/retry identity; it requires a prior verified QUERY. */
uint64_t board_recovery_boot_current(void);
#endif
