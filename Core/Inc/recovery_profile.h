#ifndef RECOVERY_PROFILE_H
#define RECOVERY_PROFILE_H

#include <stddef.h>
#include <stdint.h>

/* Private recovery-bridge control, outside the SEDSnet type namespace.
 * Kind12 belongs to older framing negotiation; current framing uses15.
 * A profile response reports application/UART configuration, not live modem
 * AT settings. It cannot change any configuration or operate an output. */
#define RECOVERY_PROFILE_KIND 14U
#define RECOVERY_PROFILE_QUERY_SIZE 26U
#define RECOVERY_PROFILE_REPLY_SIZE 42U
#define RECOVERY_PROFILE_NONCE_SIZE 16U
#define RECOVERY_PROFILE_FLOW_CONTROL 1U
#define RECOVERY_PROFILE_LORA_SCHEDULER 2U
#define RECOVERY_PROFILE_UART_FORMAT_MISMATCH 4U

typedef struct {
  uint32_t serial_baud;
  uint32_t usable_air_bps;
  uint8_t rf_share_percent;
  uint16_t frame_overhead_bytes;
  uint16_t packet_burst_ms;
  uint16_t packet_gap_ms;
  uint8_t flags;
} recovery_profile_config_t;

static inline uint32_t recovery_profile_crc32(const uint8_t *data, size_t size)
{
  uint32_t crc = UINT32_MAX;
  for (size_t i = 0U; i < size; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0U; bit < 8U; ++bit)
      crc = (crc >> 1U) ^ (0xEDB88320UL & (0U - (crc & 1U)));
  }
  return ~crc;
}

static inline uint32_t recovery_profile_read_u32(const uint8_t *data)
{
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
      ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static inline void recovery_profile_write_u32(uint8_t *data, uint32_t value)
{
  for (unsigned i = 0U; i < 4U; ++i)
    data[i] = (uint8_t)(value >> (8U * i));
}

static inline void recovery_profile_write_u16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
}

static inline int recovery_profile_is_query(
    const uint8_t *query, size_t size, uint8_t command)
{
  if (!query || !command || size != RECOVERY_PROFILE_QUERY_SIZE ||
      query[0] != 'R' || query[1] != 'R' || query[2] != 1U ||
      query[3] != RECOVERY_PROFILE_KIND || query[4] != 1U || query[5] != 0U)
    return 0;
  if (recovery_profile_crc32(query, size - 4U) !=
      recovery_profile_read_u32(query + size - 4U))
    return 0;
  uint8_t nonce_nonzero = 0U;
  for (size_t i = 0U; i < RECOVERY_PROFILE_NONCE_SIZE; ++i)
    nonce_nonzero |= query[6U + i];
  return nonce_nonzero != 0U;
}

/* RR v1 kind14, body v1/query0/nonzero128-bit challenge, then CRC32.
 * Reply body: v1/reply1/challenge/baud:u32/usable_bps:u32/share:u8/
 * overhead:u16/burst_ms:u16/gap_ms:u16/flags:u8, all little endian.
 * Exact sizes and reserved-bit validation prevent a reply or another control
 * envelope from being treated as a request. No heap, I/O or mutable state. */
static inline int recovery_profile_build_reply(
    const uint8_t *query, size_t size, uint8_t command,
    const recovery_profile_config_t *config,
    uint8_t reply[RECOVERY_PROFILE_REPLY_SIZE])
{
  if (!config || !reply || !recovery_profile_is_query(query, size, command))
    return 0;
  if (config->serial_baud == 0U ||
      config->serial_baud > 2000000U || config->usable_air_bps < 1200U ||
      config->usable_air_bps > 1000000U || config->rf_share_percent == 0U ||
      config->rf_share_percent > 100U || config->frame_overhead_bytes > 256U ||
      config->packet_burst_ms == 0U || config->packet_burst_ms > 500U ||
      config->packet_gap_ms == 0U || config->packet_gap_ms > 1000U ||
      (config->flags & ~(RECOVERY_PROFILE_FLOW_CONTROL |
                         RECOVERY_PROFILE_LORA_SCHEDULER |
                         RECOVERY_PROFILE_UART_FORMAT_MISMATCH)) != 0U)
    return 0;
  reply[0] = 'R'; reply[1] = 'R'; reply[2] = 1U;
  reply[3] = RECOVERY_PROFILE_KIND; reply[4] = 1U; reply[5] = 1U;
  for (size_t i = 0U; i < RECOVERY_PROFILE_NONCE_SIZE; ++i)
    reply[6U + i] = query[6U + i];
  recovery_profile_write_u32(reply + 22U, config->serial_baud);
  recovery_profile_write_u32(reply + 26U, config->usable_air_bps);
  reply[30] = config->rf_share_percent;
  recovery_profile_write_u16(reply + 31U, config->frame_overhead_bytes);
  recovery_profile_write_u16(reply + 33U, config->packet_burst_ms);
  recovery_profile_write_u16(reply + 35U, config->packet_gap_ms);
  reply[37] = config->flags;
  recovery_profile_write_u32(reply + 38U, recovery_profile_crc32(reply, 38U));
  return 1;
}

#endif
