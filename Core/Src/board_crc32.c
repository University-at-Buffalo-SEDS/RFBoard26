/* SEDSnet frame CRC-32 on the STM32 CRC unit (G4, U5 and H5 share the
 * register layout used here).
 *
 * IEEE CRC-32 is the reflected form of polynomial 0x04C11DB7. The unit shifts
 * MSB first, so its register S is the bit reversal of the reflected state R:
 *   - REV_IN = 01 (bit reversal within each byte) feeds every byte LSB first;
 *     an 8-bit DR write processes one byte, a 32-bit write processes bits
 *     31..0, so a little-endian word is byte-swapped (REV) to put byte 0 first.
 *   - REV_OUT = 1 makes DR read back R = rbit(S).
 *   - A finished zlib CRC c has R = ~c, so a call resumes from
 *     INIT = rbit(~c) (0 -> 0xFFFFFFFF, the IEEE initial value) and returns
 *     ~DR (the final XOR).
 *   - POLYSIZE = 00 selects a 32-bit polynomial; POL = 0x04C11DB7.
 * The unit inserts bus wait states while it computes, so no polling is needed.
 *
 * Concurrency: one try-lock covers a whole hardware call. A caller that finds
 * it held (another thread, or an ISR that preempted a hardware call) computes
 * in software instead, so the function never blocks or masks interrupts and
 * the hardware sequence is never interleaved. */
#include "board_crc32.h"

#include <stdatomic.h>
#include <string.h>

#include "sedsnet.h"

/* CRC_CR bits; identical on STM32G4, STM32U5 and STM32H5. */
#define BOARD_CRC_CR_RESET 0x01UL
#define BOARD_CRC_CR_REV_IN_BYTE 0x20UL
#define BOARD_CRC_CR_REV_OUT 0x80UL
#define BOARD_CRC_POLY 0x04C11DB7UL
#define BOARD_CRC_CR (BOARD_CRC_CR_REV_IN_BYTE | BOARD_CRC_CR_REV_OUT)

#ifdef BOARD_CRC32_HOST_MODEL
#include "board_crc32_host_model.h"
#else
#include "main.h"
_Static_assert(BOARD_CRC_CR_RESET == CRC_CR_RESET, "CRC_CR.RESET");
_Static_assert(BOARD_CRC_CR_REV_IN_BYTE == CRC_CR_REV_IN_0, "CRC_CR.REV_IN");
_Static_assert(BOARD_CRC_CR_REV_OUT == CRC_CR_REV_OUT, "CRC_CR.REV_OUT");
_Static_assert(CRC_CR_POLYSIZE_Pos == 3U, "CRC_CR.POLYSIZE");
#define BOARD_CRC_CLOCK_ENABLE() __HAL_RCC_CRC_CLK_ENABLE()
#define BOARD_CRC_WRITE_POL(v) (CRC->POL = (v))
#define BOARD_CRC_WRITE_INIT(v) (CRC->INIT = (v))
#define BOARD_CRC_WRITE_CR(v) (CRC->CR = (v))
#define BOARD_CRC_WRITE_DR32(v) (CRC->DR = (v))
#define BOARD_CRC_WRITE_DR8(v) (*(volatile uint8_t *)(volatile void *)&CRC->DR = (v))
#define BOARD_CRC_READ_DR() (CRC->DR)
#define BOARD_CRC_RBIT(v) __RBIT(v)
#define BOARD_CRC_REV(v) __REV(v)
#endif

volatile uint8_t g_board_crc32_state;
static atomic_flag s_crc_busy = ATOMIC_FLAG_INIT;

/* Reflected 0xEDB88320 table, four bits per step (64 bytes of flash). */
static const uint32_t k_crc_nibble[16] = {
    0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
    0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
    0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
    0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL,
};

uint32_t board_crc32_software(uint32_t crc, const uint8_t *data, size_t len)
{
  crc = ~crc;
  while (len-- != 0U) {
    crc ^= *data++;
    crc = (crc >> 4) ^ k_crc_nibble[crc & 15U];
    crc = (crc >> 4) ^ k_crc_nibble[crc & 15U];
  }
  return ~crc;
}

/* Caller owns the unit. */
static uint32_t board_crc32_hw(uint32_t crc, const uint8_t *data, size_t len)
{
  BOARD_CRC_WRITE_INIT(BOARD_CRC_RBIT(~crc));
  BOARD_CRC_WRITE_CR(BOARD_CRC_CR | BOARD_CRC_CR_RESET);
  while (len != 0U && ((uintptr_t)data & 3U) != 0U) {
    BOARD_CRC_WRITE_DR8(*data++);
    --len;
  }
  for (; len >= 4U; len -= 4U, data += 4U) {
    uint32_t word;
    memcpy(&word, data, sizeof word);
    BOARD_CRC_WRITE_DR32(BOARD_CRC_REV(word));
  }
  while (len-- != 0U) BOARD_CRC_WRITE_DR8(*data++);
  return ~BOARD_CRC_READ_DR();
}

uint32_t board_crc32(uint32_t crc, const uint8_t *data, size_t len)
{
  if (len == 0U) return crc;
  if (atomic_flag_test_and_set_explicit(&s_crc_busy, memory_order_acquire)) {
    return board_crc32_software(crc, data, len);
  }
  crc = board_crc32_hw(crc, data, len);
  atomic_flag_clear_explicit(&s_crc_busy, memory_order_release);
  return crc;
}

static bool board_crc32_self_test(void)
{
  static const uint8_t k_vec[16] __attribute__((aligned(4))) = {
      '1', '2', '3', '4', '5', '6', '7', '8', '9', 'S', 'E', 'D', 'S', 0x00, 0x80, 0xFF};
  const uint32_t check = 0xCBF43926UL;
  if (board_crc32_software(0U, k_vec, 9U) != check) return false;
  if (board_crc32_hw(0U, k_vec, 9U) != check) return false;
  if (board_crc32_hw(board_crc32_hw(0U, k_vec, 3U), k_vec + 3, 6U) != check) return false;
  for (uint32_t off = 0U; off < 4U; ++off) {
    for (uint32_t len = 0U; len <= 12U; ++len) {
      const uint32_t seed = 0x9E3779B9UL * (off * 13U + len + 1U);
      if (board_crc32_hw(seed, k_vec + off, len) !=
          board_crc32_software(seed, k_vec + off, len)) {
        return false;
      }
    }
  }
  return true;
}

bool board_crc32_install(void)
{
  BOARD_CRC_CLOCK_ENABLE();
  BOARD_CRC_WRITE_POL(BOARD_CRC_POLY);
  BOARD_CRC_WRITE_CR(BOARD_CRC_CR | BOARD_CRC_CR_RESET);
  const bool ok = board_crc32_self_test();
  g_board_crc32_state = ok ? 1U : 2U;
  if (ok) seds_set_crc32_fn(board_crc32);
  return ok;
}
