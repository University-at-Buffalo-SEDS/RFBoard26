#ifndef BOARD_RECOVERY_FLOOR_H
#define BOARD_RECOVERY_FLOOR_H
#include <stdint.h>
/* No implicit clear. Corrupt/storage-failed floor rejects commands; QUERY lives.
 * Caller must separately validate canonical current network time and source.
 * Every time here is a station (time_epoch, timestamp) key (network_stamp.h),
 * stored in the floor record's 8-byte time field: a floor written before
 * SEDSnet 6 holds a bare stamp and reads as epoch 0, and a request made after
 * a backward station clock step (a newer epoch) is later than it. */
uint8_t board_recovery_floor_active(void);
/* May perform storage access: never call under control-owner mutex. */
uint8_t board_recovery_floor_read_cutoff(uint64_t *cutoff_ms);
uint8_t board_recovery_floor_allows_recovery_time(uint64_t issuer_session, uint64_t source_time_ms);
uint8_t board_recovery_floor_is_healthy(void);
uint8_t board_recovery_command_time_allowed(uint64_t canonical_time_ms);
/* Explicit authorized RECOVER only, after boot/epoch/nonce/fresh-time checks.
 * Repairs require a strictly later (epoch, timestamp) key than every
 * verified/attempted one, across all issuers; changing issuer never clears or
 * lowers the floor.
 * Returns 1 only after persistent write + exact readback; must precede latches. */
uint8_t board_recovery_floor_commit(uint64_t issuer_session, uint64_t source_time_ms);
uint8_t board_recovery_floor_accept_bootstrap(uint64_t original_source_time_ms);
uint8_t board_recovery_floor_note_bootstrap_attempt(uint64_t original_source_time_ms);
uint8_t board_recovery_floor_prime_boot(void);
uint8_t board_recovery_floor_cached_health(void);
uint8_t board_recovery_floor_cached_cutoff(uint64_t *cutoff);
#endif
