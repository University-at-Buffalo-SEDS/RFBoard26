#ifndef BOARD_COMMISSIONING_H
#define BOARD_COMMISSIONING_H
#include <stdbool.h>
#include <stdint.h>
#include "board_recovery_wire.h"
typedef bool(*CommissionReply)(void*,const BoardRecoveryRequest*,uint8_t);
typedef uint64_t(*CommissionClock)(void*);
/* Prethreads, checked exclusive storage phase; no writes or output changes. */
void board_commissioning_probe_boot(void);
bool board_commissioning_pending(void);
/* Known genuine failed FIRST/MAX, no claimed durable/ready identity. */
bool board_commissioning_uncertain_first(void);
uint64_t board_commissioning_query_generation(void);
uint8_t board_commissioning_query_result(void);
/* Exactly one existing SDK/router owner calls stage/query/service at runtime.
 * The FC priority0 owner shares only its own IRQ mailbox, never these jobs. */
bool board_commissioning_note_query(const BoardRecoveryRequest*,uint64_t,uint64_t,uint32_t);
int32_t board_commissioning_stage(const BoardRecoveryRequest*,uint64_t,uint64_t,uint32_t,void*,CommissionReply,CommissionClock);
void board_commissioning_service(void);
/* Fixed concrete domain owner. No state/IRQ lock crosses storage or SDK. */
int32_t board_recovery_commission_prepare(uint32_t expected_epoch);
bool board_recovery_commission_held(uint32_t expected_epoch);
int32_t board_recovery_commission_apply(uint32_t expected_epoch);
bool board_recovery_commission_applied(uint32_t expected_epoch,uint32_t*actual_epoch);
bool board_recovery_commission_finish(uint32_t expected_epoch,uint32_t actual_epoch);
void board_recovery_commission_retain(void);
#endif
