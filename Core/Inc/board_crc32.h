#ifndef BOARD_CRC32_H
#define BOARD_CRC32_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SEDSnet frame CRC-32 (IEEE, zlib-incremental) on the STM32 CRC unit.
 *
 * board_crc32_install() enables and configures the unit, self-tests it and
 * only then installs board_crc32() with seds_set_crc32_fn(). On a failed
 * self-test SEDSnet keeps its software default. Call it once from main()
 * before the scheduler starts and before any router exists. */
bool board_crc32_install(void);

/* SedsCrc32Fn: CRC of the bytes already covered by `crc` followed by `len`
 * bytes at `data`. Never blocks: if the unit is busy (another thread, or an
 * ISR that preempted a hardware call), the call is computed in software. */
uint32_t board_crc32(uint32_t crc, const uint8_t *data, size_t len);

/* The software path with the same semantics. */
uint32_t board_crc32_software(uint32_t crc, const uint8_t *data, size_t len);

/* 0 before install, 1 hardware installed, 2 self-test failed (software). */
extern volatile uint8_t g_board_crc32_state;

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CRC32_H */
