#ifndef BOARD_RECOVERY_WIRE_H
#define BOARD_RECOVERY_WIRE_H
#include <stddef.h>
#include <stdint.h>
#define BOARD_RECOVERY_REQUEST_SIZE 32U
#define BOARD_RECOVERY_STATUS_SIZE 40U
#define BOARD_RECOVERY_VERSION 1U
enum { BOARD_RECOVERY_QUERY=0U, BOARD_RECOVERY_RECOVER=1U };
enum { BOARD_RECOVERY_RESULT_OK=0U, BOARD_RECOVERY_RESULT_PENDING=1U,
 BOARD_RECOVERY_RESULT_UNSUPPORTED=2U, BOARD_RECOVERY_RESULT_UNAUTHORIZED=3U,
 BOARD_RECOVERY_RESULT_STALE=4U, BOARD_RECOVERY_RESULT_UNSAFE=5U,
 BOARD_RECOVERY_RESULT_INVALID=6U, BOARD_RECOVERY_RESULT_IO=7U };
typedef struct {
 uint8_t operation, desired_state;
 uint64_t nonce, expected_boot_generation;
 uint32_t expected_abort_epoch;
 uint64_t issuer_session;
} BoardRecoveryRequest;
/* Strict shape validation only; caller must authorize source and validate the
 * live boot/abort epoch and canonical source-time before any state change. */
uint8_t board_recovery_decode_request(const uint8_t *wire, size_t length,
                                     BoardRecoveryRequest *request);
void board_recovery_encode_status(uint8_t wire[BOARD_RECOVERY_STATUS_SIZE],
 uint8_t result, uint8_t actual_state, uint8_t flags,
 const BoardRecoveryRequest *request, uint64_t boot_generation,
 uint32_t abort_epoch, uint32_t uptime_ms, uint32_t commanded_outputs);
#endif
