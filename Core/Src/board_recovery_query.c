#include "board_recovery.h"
#include "flight_state_cache.h"
#include <limits.h>
/* All entry points execute on the existing router/cache owner. Epoch is bound
 * to the verified unique boot generation; no physical output domain exists
 * here. In particular PB power rails and their configuration are untouched. */
static uint32_t recovery_epoch = 1U;
static uint32_t completed_epoch;
int32_t board_recovery_snapshot(BoardRecoverySnapshot *out)
{
    if (out == 0) return -10;
    *out = (BoardRecoverySnapshot){.actual_state = 255U,
        .flags = BOARD_RECOVERY_SUPPORTED, .abort_epoch = recovery_epoch};
    if (flight_state_cache_has_value()) {
        out->actual_state = flight_state_cache_value();
        out->flags |= BOARD_RECOVERY_STATE_VALID;
    }
    return 0;
}
int32_t board_recovery_apply_safe(uint8_t desired, uint32_t expected,
                                BoardRecoverySnapshot *out)
{
    if (out == 0 || desired != 0U || expected == 0U || expected == UINT32_MAX)
        return -10;
    if (completed_epoch == expected && recovery_epoch == expected + 1U &&
        flight_state_cache_has_value() && flight_state_cache_value() == 0U)
        return board_recovery_snapshot(out);
    if (expected != recovery_epoch) {
        (void)board_recovery_snapshot(out);
        return -10;
    }
    if (flight_state_cache_reset_startup() != 0) {
        (void)board_recovery_snapshot(out);
        return -9;
    }
    completed_epoch = expected;
    ++recovery_epoch;
    return board_recovery_snapshot(out);
}
