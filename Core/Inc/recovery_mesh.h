#ifndef RECOVERY_MESH_H
#define RECOVERY_MESH_H
#include <stddef.h>
#include <stdint.h>

/* Ground/RRLS envelopes never enter the SEDSnet side-template namespace.
 * Only kinds 5/6 address the avionics modem; all other kinds are host-only. */
static inline int recovery_mesh_uplink(const uint8_t **data, size_t *len,
                                       uint8_t *command) {
  const uint8_t *p = *data;
  if (!*command || *len < 2U || p[0] != 'R' || p[1] != 'R') return 1;
  if (*len < 9U || p[2] != 1U || (p[3] != 5U && p[3] != 6U)) return 0;
  const size_t end = *len - 4U;
  uint32_t crc = UINT32_MAX;
  for (size_t i = 0; i < end; ++i) {
    crc ^= p[i];
    for (unsigned b = 0; b < 8U; ++b)
      crc = (crc >> 1) ^ (0xEDB88320UL & (0U - (crc & 1U)));
  }
  const uint32_t expected = (uint32_t)p[end] | ((uint32_t)p[end+1U] << 8) |
      ((uint32_t)p[end+2U] << 16) | ((uint32_t)p[end+3U] << 24);
  if (~crc != expected) return 0;
  *command = p[3] == 6U;
  *data = p + 4U;
  *len = end - 4U;
  return 1;
}
#endif
