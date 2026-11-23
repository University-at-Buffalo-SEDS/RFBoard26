#ifndef RECOVERY_MESH_H
#define RECOVERY_MESH_H
#include <stddef.h>
#include <stdint.h>
#include "recovery_profile.h"

/* RR v2 ground/RRLS envelopes (GS backend rrls.rs). One marker byte in
 * 0xE0..0xFF, a range no SEDSnet first byte and no "RS" window uses:
 *  - CRC form 0xE0+kind (kinds 0..16): payload, CRC32 of marker+payload.
 *  - Self-checked form (kinds 1..6 only, markers F3 F5 F6 F9 FA FC): an SDK
 *    frame whose own trailing CRC32 covers it, no outer CRC. Each marker's
 *    low nibble has two bits set, so one bit error never yields another
 *    self-checked marker (bit 4 gives the CRC form, whose CRC then fails). */
#define RECOVERY_MESH_MARKER_MIN 0xE0U
#define RECOVERY_MESH_CRC_MARKER(kind) (RECOVERY_MESH_MARKER_MIN + (kind))
#define RECOVERY_MESH_SELF_DATA 0xFAU    /* kind5 AV-bay data */
#define RECOVERY_MESH_SELF_COMMAND 0xFCU /* kind6 AV-bay command */

static inline int recovery_mesh_self_checked(uint8_t marker) {
  return (marker & 0xF0U) == 0xF0U && ((0x1668U >> (marker & 0x0FU)) & 1U);
}

/* Ground/RRLS envelopes never enter the SEDSnet side-template namespace.
 * Kinds5/6 address avionics data; kind14 admits only a validated read-only
 * application-profile query. Other kinds remain host-only. */
static inline int recovery_mesh_uplink(const uint8_t **data, size_t *len,
                                       uint8_t *command) {
  const uint8_t *p = *data;
  if (!*command || *len < 1U || p[0] < RECOVERY_MESH_MARKER_MIN) return 1;
  if (p[0] == RECOVERY_PROFILE_MARKER)
    return recovery_profile_is_query(p, *len, *command);
  const size_t self = p[0] == RECOVERY_MESH_SELF_DATA ||
                      p[0] == RECOVERY_MESH_SELF_COMMAND;
  if (*len < 5U || (!self && p[0] != RECOVERY_MESH_CRC_MARKER(5U) &&
                    p[0] != RECOVERY_MESH_CRC_MARKER(6U)))
    return 0;
  const size_t end = *len - 4U;
  if (recovery_profile_crc32(p + self, end - self) !=
      recovery_profile_read_u32(p + end))
    return 0;
  *command = p[0] == RECOVERY_MESH_SELF_COMMAND ||
             p[0] == RECOVERY_MESH_CRC_MARKER(6U);
  *data = p + 1U;
  *len = self ? *len - 1U : end - 1U;
  return 1;
}
#endif
