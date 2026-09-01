#ifndef SIM_NETWORK_PROBE_H
#define SIM_NETWORK_PROBE_H

#include <stddef.h>
#include <stdint.h>

extern volatile uint32_t g_telemetry_peer_mask;
extern volatile uint32_t g_sim_heartbeat_attempts;
extern volatile uint32_t g_sim_heartbeat_ok;
extern volatile uint32_t g_sim_heartbeat_fail;
extern volatile uint32_t g_sim_heartbeat_wire_tx;

static inline uint32_t sim_probe_peer_bit_sender(const char *sender,
                                                 size_t len) {
#ifdef SEDS_FIRMWARE_SIM_TEST
  static const struct { const char *name; size_t len; uint32_t bit; } peers[] = {
      {"RF", 2U, 1U}, {"PB", 2U, 2U}, {"FC", 2U, 4U}, {"GB", 2U, 8U},
      {"AB", 2U, 16U}, {"VB", 2U, 32U}, {"DAQ", 3U, 64U}};
  for (size_t i = 0U; sender != NULL && i < sizeof(peers) / sizeof(peers[0]); ++i) {
    size_t j = 0U;
    if (len != peers[i].len) continue;
    while (j < len && sender[j] == peers[i].name[j]) j++;
    if (j == len) return peers[i].bit;
  }
#else
  (void)sender; (void)len;
#endif
  return 0U;
}

static inline SedsResult sim_probe_heartbeat_handler(const SedsPacketView *pkt,
                                                     void *user) {
  (void)user;
#ifdef SEDS_FIRMWARE_SIM_TEST
  if (pkt != NULL && pkt->ty == (uint32_t)SEDS_DT_HEARTBEAT) {
    g_telemetry_peer_mask |= sim_probe_peer_bit_sender(pkt->sender, pkt->sender_len);
  }
#else
  (void)pkt;
#endif
  return SEDS_OK;
}

static inline uint8_t sim_probe_read_uleb(const uint8_t *data, size_t len,
                                          size_t *offset, uint64_t *value) {
  uint64_t out = 0U;
  uint32_t shift = 0U;
  for (uint32_t i = 0U; i < 10U && *offset < len; ++i) {
    const uint8_t byte = data[(*offset)++];
    out |= ((uint64_t)(byte & 0x7FU)) << shift;
    if ((byte & 0x80U) == 0U) {
      *value = out;
      return 1U;
    }
    shift += 7U;
  }
  return 0U;
}

static inline uint32_t sim_probe_sender_address(const char *sender) {
  uint64_t hash = 0xA6D38C214B7F19E5ULL;
  while (*sender != '\0') {
    hash ^= (uint8_t)*sender++;
    hash *= 0x9E3779B1ULL;
    hash ^= hash >> 27U;
  }
  const uint32_t address = (uint32_t)hash;
  return address == 0U ? 1U : address;
}

static inline uint32_t sim_probe_packed_data_type(const uint8_t *data,
                                                  size_t len) {
  size_t offset = 2U;
  uint64_t data_type = 0U;
  if (data != NULL && len >= 3U &&
      sim_probe_read_uleb(data, len, &offset, &data_type) &&
      data_type <= UINT32_MAX) {
    return (uint32_t)data_type;
  }
  return UINT32_MAX;
}

static inline void sim_probe_observe_can_tx(const uint8_t *data, size_t len) {
#ifdef SEDS_FIRMWARE_SIM_TEST
  if (sim_probe_packed_data_type(data, len) == (uint32_t)SEDS_DT_HEARTBEAT) {
    g_sim_heartbeat_wire_tx++;
  }
#else
  (void)data;
  (void)len;
#endif
}

/* Allocation-free observation of SEDSNet v4's packed source-address field. */
static inline uint32_t sim_probe_peer_bit_packed(const uint8_t *data,
                                                 size_t len) {
#ifdef SEDS_FIRMWARE_SIM_TEST
  static const struct {
    const char *sender;
    uint32_t bit;
  } peers[] = {{"RF", 1U},  {"PB", 2U},  {"FC", 4U},  {"GB", 8U},
               {"AB", 16U}, {"VB", 32U}, {"DAQ", 64U}};
  size_t offset = 0U;
  uint64_t ignored = 0U;
  uint64_t source_address = 0U;
  if (data == NULL || len < 3U) return 0U;
  const uint8_t flags = data[offset++];
  offset++; /* selected endpoint count */
  if (!sim_probe_read_uleb(data, len, &offset, &ignored) ||
      !sim_probe_read_uleb(data, len, &offset, &ignored) ||
      !sim_probe_read_uleb(data, len, &offset, &ignored)) return 0U;
  if ((flags & 0x08U) != 0U &&
      !sim_probe_read_uleb(data, len, &offset, &ignored)) return 0U;
  if (!sim_probe_read_uleb(data, len, &offset, &source_address)) return 0U;
  for (size_t i = 0U; i < sizeof(peers) / sizeof(peers[0]); ++i) {
    if ((uint32_t)source_address == sim_probe_sender_address(peers[i].sender)) {
      return peers[i].bit;
    }
  }
#else
  (void)data;
  (void)len;
#endif
  return 0U;
}

static inline void sim_probe_observe_packed(const uint8_t *data, size_t len) {
#ifdef SEDS_FIRMWARE_SIM_TEST
  g_telemetry_peer_mask |= sim_probe_peer_bit_packed(data, len);
#else
  (void)data;
  (void)len;
#endif
}

/* Emit real end-to-end traffic for linked-bay validation. Unlike aggregated
 * topology advertisements, this packet retains the originating board ID
 * through every relay, so receivers can certify multi-hop delivery. */
static inline void sim_probe_emit_heartbeat(SedsRouter *router,
                                            uint64_t now_ms) {
#ifdef SEDS_FIRMWARE_SIM_TEST
  static uint32_t next_emit_ms = 2000U;
  static uint8_t emit_count = 0U;
  static uint32_t service_cycles = 0U;
  static const uint8_t empty_payload = 0U;
  service_cycles++;
  if (router != NULL && emit_count < 4U &&
      (now_ms >= next_emit_ms ||
       service_cycles >= (1U + ((uint32_t)emit_count * 75U)))) {
    g_sim_heartbeat_attempts++;
    if (seds_router_log_bytes_ex(router, SEDS_DT_HEARTBEAT, &empty_payload,
                                 0U, NULL, 1) == SEDS_OK) {
      emit_count++;
      next_emit_ms = (uint32_t)now_ms + 100U;
      g_sim_heartbeat_ok++;
    } else {
      next_emit_ms = (uint32_t)now_ms + 50U;
      g_sim_heartbeat_fail++;
    }
  }
#else
  (void)router;
  (void)now_ms;
#endif
}
#endif
