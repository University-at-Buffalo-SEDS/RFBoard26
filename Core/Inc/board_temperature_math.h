#ifndef BOARD_TEMPERATURE_MATH_H
#define BOARD_TEMPERATURE_MATH_H
#include <stdint.h>
#include <stdbool.h>
/* Ratio compensation works at either ADC resolution: the factory VREF code
 * converts the measured temperature code to the factory calibration scale. */
static inline bool board_temperature_convert(uint32_t temp, uint32_t vref,
    uint16_t vref_cal, uint16_t cal1, uint16_t cal2,
    int32_t cal1_c, int32_t cal2_c, int16_t *out)
{
  if (!out || !vref || !temp || !vref_cal || vref_cal == UINT16_MAX ||
      !cal1 || cal1 == UINT16_MAX || cal2 <= cal1 || cal2 == UINT16_MAX) return false;
  const int64_t code = ((int64_t)temp * vref_cal + vref / 2U) / vref;
  const int64_t c = cal1_c * 100 + (code - cal1) * (cal2_c - cal1_c) * 100 / (cal2 - cal1);
  if (c < -4000 || c > 15000) return false;
  *out = (int16_t)c;
  return true;
}
/* Bits 0..7 retain existing diagnostics. Bit 8 means valid; bits 9..15
 * carry age in 100ms units; bits 16..31 carry signed centidegrees C.
 * Report size/schema/version remain unchanged for mixed firmware. */
static inline uint32_t board_temperature_pack(int16_t centi_c, uint32_t age_ms)
{
  if (age_ms > 6000U || centi_c < -4000 || centi_c > 15000) return 0U;
  return ((uint32_t)(uint16_t)centi_c << 16U) | 0x100U |
         (((age_ms + 99U) / 100U) << 9U);
}
#endif
