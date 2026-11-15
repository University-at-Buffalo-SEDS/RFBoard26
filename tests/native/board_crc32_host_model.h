/* Host register model hooks for Core/Src/board_crc32.c (BOARD_CRC32_HOST_MODEL).
 * tests/native/board_crc32_test.c implements an STM32 CRC unit behind them. */
#ifndef BOARD_CRC32_HOST_MODEL_H
#define BOARD_CRC32_HOST_MODEL_H

#include <stdint.h>

void crc_model_clock_enable(void);
void crc_model_write_pol(uint32_t value);
void crc_model_write_init(uint32_t value);
void crc_model_write_cr(uint32_t value);
void crc_model_write_dr(uint32_t value, unsigned width);
uint32_t crc_model_read_dr(void);

static inline uint32_t crc_model_rbit(uint32_t v)
{
  uint32_t r = 0U;
  for (unsigned i = 0U; i < 32U; ++i) {
    r = (r << 1) | (v & 1U);
    v >>= 1;
  }
  return r;
}

static inline uint32_t crc_model_rev(uint32_t v)
{
  return (v >> 24) | ((v >> 8) & 0xFF00U) | ((v << 8) & 0xFF0000U) | (v << 24);
}

#define BOARD_CRC_CLOCK_ENABLE() crc_model_clock_enable()
#define BOARD_CRC_WRITE_POL(v) crc_model_write_pol(v)
#define BOARD_CRC_WRITE_INIT(v) crc_model_write_init(v)
#define BOARD_CRC_WRITE_CR(v) crc_model_write_cr(v)
#define BOARD_CRC_WRITE_DR32(v) crc_model_write_dr((v), 32U)
#define BOARD_CRC_WRITE_DR8(v) crc_model_write_dr((v), 8U)
#define BOARD_CRC_READ_DR() crc_model_read_dr()
#define BOARD_CRC_RBIT(v) crc_model_rbit(v)
#define BOARD_CRC_REV(v) crc_model_rev(v)

#endif
