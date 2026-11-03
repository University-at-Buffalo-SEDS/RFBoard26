#include "board_recovery.h"
#include "board_commissioning.h"
#include "persistent_store.h"
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
    /* Explicit application metadata inhibit, never a physical-output claim. */
    if(persistent_store_commission_pending())out->flags|=BOARD_RECOVERY_ABORT_LATCHED;
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

/* Single existing router/cache owner only. These endpoints own no actuators,
 * power rails, acquisition restart or hardware-reset operation. */
int32_t board_recovery_commission_prepare(uint32_t expected){
 if(!expected||expected==UINT32_MAX||expected!=recovery_epoch)return -10;
 persistent_store_hold_commissioning();return 0;
}
bool board_recovery_commission_held(uint32_t expected){return expected==recovery_epoch&&persistent_store_commission_pending();}
int32_t board_recovery_commission_apply(uint32_t expected){BoardRecoverySnapshot snapshot;return board_recovery_apply_safe(0,expected,&snapshot);}
bool board_recovery_commission_applied(uint32_t expected,uint32_t*actual){
 bool valid=actual&&expected&&expected!=UINT32_MAX&&completed_epoch==expected&&recovery_epoch==expected+1U&&persistent_store_commission_pending()&&flight_state_cache_has_value()&&flight_state_cache_value()==0;
 if(valid){*actual=recovery_epoch;}return valid;
}
bool board_recovery_commission_finish(uint32_t expected,uint32_t actual){uint32_t live=0;return board_recovery_commission_applied(expected,&live)&&live==actual&&persistent_store_publish_commission_complete();}
void board_recovery_commission_retain(void){persistent_store_hold_commissioning();completed_epoch=0;if(recovery_epoch!=UINT32_MAX)++recovery_epoch;}
