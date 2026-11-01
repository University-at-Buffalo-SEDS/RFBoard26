#ifndef BOARD_RECOVERY_H
#define BOARD_RECOVERY_H
#include <stdint.h>
/* Application protocol only. Wire encoding is explicit LE, never memcpy of this struct. */
enum {
 BOARD_RECOVERY_ABORT_LATCHED=1u<<0,
 BOARD_RECOVERY_SAFE_OUTPUTS=1u<<1,
 BOARD_RECOVERY_SUPPORTED=1u<<2,
 BOARD_RECOVERY_STATE_VALID=1u<<3,
 BOARD_RECOVERY_OUTPUTS_VALID=1u<<4
};
typedef struct {
 uint8_t actual_state;
 uint8_t flags;
 uint16_t reserved;
 uint32_t abort_epoch;
 uint32_t actual_outputs_mask;
} BoardRecoverySnapshot;
/* SEDS_OK (0) or negative SedsResult. Snapshot must be a serialized actual readback. */
int32_t board_recovery_snapshot(BoardRecoverySnapshot *out);
/* Manual operator request only. Allowed desired_state: Startup(0), Idle(1).
 * Under the board's state lock, validate expected_abort_epoch, check safe outputs,
 * purge queued commands, verify explicit persistent latch clear, update state and
 * advance epoch. A concurrent abort must invalidate this request. Never replay
 * actuator/valve/igniter/launch/pyro commands or claim unsupported hardware safe.
 * Return a fresh readback even on rejection where possible. */
int32_t board_recovery_apply_safe(uint8_t desired_state,
                                uint32_t expected_abort_epoch,
                                BoardRecoverySnapshot *out);

/* Control-owner binding. Functions run under the state mutex and must perform
 * bounded register/RAM access only: no SDK, storage, status sends or waits. */
void board_recovery_bind_owner(uint8_t (*read_outputs)(uint32_t *),
                              uint8_t (*prepare)(uint8_t), void (*finish)(void));
void thread_comm_service_board_recovery(void);
#endif
