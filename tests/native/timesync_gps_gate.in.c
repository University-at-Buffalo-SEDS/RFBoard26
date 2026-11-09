/* RF time-source gate with the actual SDK. The production telemetry.c
 * timesync functions are inserted at test time; a relayed consumer (an
 * Actuator Board behind the gateway) shares one simulated monotonic clock.
 * Without a GPS fix RF must not announce, so the consumer never anchors to
 * RF uptime; with a fix RF leads with GPS UTC; after the bounded holdover it
 * stops announcing again. */
#include "sedsnet.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TX_TIMER_TICKS_PER_SECOND 1000U
#define TELEMETRY_ENABLED 1
#define SEDS_FIRMWARE_SIM_TEST 0
#define TELEMETRY_TIMESYNC_MASTER_PRIO (1)
#define TELEMETRY_TIMESYNC_ROLE_CONSUMER 0U
#define TELEMETRY_TIMESYNC_ROLE_SOURCE 1U
#define TELEMETRY_TIMESYNC_SOURCE_TIMEOUT_MS 5000U
#define TELEMETRY_TIMESYNC_ANNOUNCE_INTERVAL_MS 2000U
#define TELEMETRY_TIMESYNC_REQUEST_INTERVAL_MS 2000U
@@HOLDOVER@@

typedef struct {SedsRouter *r; uint8_t created; uint64_t start_time;} RouterState;
static RouterState g_router;
static uint8_t g_local_unix_valid;
static uint64_t g_local_unix_ms;
static uint32_t g_local_unix_set_ms;
static uint64_t mono_ms = 1000U;
static unsigned long tx_time_get(void) { return (unsigned long)mono_ms; }
static uint64_t now_cb(void *user) { (void)user; return mono_ms; }

@@TELEMETRY:tx_raw_now_ms_locked@@
@@TELEMETRY:telemetry_timesync_is_source@@
@@TELEMETRY:telemetry_timesync_priority@@
@@TELEMETRY:telemetry_timesync_role@@
@@TELEMETRY:telemetry_unix_ms_to_utc@@
@@TELEMETRY:telemetry_apply_local_unix_time_locked@@
@@TELEMETRY:telemetry_configure_timesync_locked@@
@@TELEMETRY:telemetry_set_unix_time_ms_locked@@
@@TELEMETRY:telemetry_expire_gps_time_locked@@

/* A bounded wire between the two routers, delivered outside SDK callbacks. */
typedef struct {uint8_t bytes[2048]; size_t len;} Frame;
typedef struct {Frame frames[64]; unsigned count;} Wire;
static Wire to_peer, to_rf;
static unsigned rf_announces;
static uint64_t last_announce_time;

static void note_rf_frame(const uint8_t *bytes, size_t len) {
  SedsOwnedPacket *pkt = seds_pkt_unpack_owned(bytes, len);
  if (!pkt) return;
  SedsPacketView view;
  /* RF also relays the consumer's own holdover announces; count RF's. */
  if (seds_owned_pkt_view(pkt, &view) == SEDS_OK && view.ty == SEDS_DT_TIME_SYNC_ANNOUNCE &&
      view.sender_len == 2U && memcmp(view.sender, "RF", 2U) == 0) {
    rf_announces++;
    assert(view.payload_len == 2U * sizeof(uint64_t));
    memcpy(&last_announce_time, view.payload + sizeof(uint64_t), sizeof(uint64_t));
  }
  seds_owned_pkt_free(pkt);
}
static SedsResult wire_tx(const uint8_t *bytes, size_t len, void *user) {
  Wire *wire = (Wire *)user;
  if (wire == &to_peer) note_rf_frame(bytes, len);
  if (wire->count == 64U || len > sizeof wire->frames[0].bytes) return SEDS_OK;
  memcpy(wire->frames[wire->count].bytes, bytes, len);
  wire->frames[wire->count++].len = len;
  return SEDS_OK;
}
static void deliver(Wire *wire, SedsRouter *dst) {
  Wire copy = *wire;
  wire->count = 0U;
  for (unsigned i = 0; i < copy.count; i++)
    (void)seds_router_receive_packed(dst, copy.frames[i].bytes, copy.frames[i].len);
}

static SedsRouter *peer;
static void run_ms(uint64_t duration, uint64_t gps_epoch_base, bool gps) {
  for (uint64_t t = 0; t < duration; t += 50U) {
    mono_ms += 50U;
    /* The GPS thread feeds each valid fix's date/time (1 Hz here). */
    if (gps && mono_ms % 1000U == 0U) telemetry_set_unix_time_ms_locked(gps_epoch_base + mono_ms);
    telemetry_expire_gps_time_locked();
    bool queued = false;
    assert(seds_router_poll_timesync(g_router.r, &queued) == SEDS_OK);
    assert(seds_router_poll_timesync(peer, &queued) == SEDS_OK);
    for (int round = 0; round < 4; round++) {
      (void)seds_router_process_all_queues(g_router.r);
      (void)seds_router_process_all_queues(peer);
      deliver(&to_peer, peer);
      deliver(&to_rf, g_router.r);
    }
  }
}

int main(int argc, char **argv) {
  assert(argc == 2);
  assert(seds_schema_register_json_file(argv[1], strlen(argv[1])) == SEDS_OK);
  g_router.r = seds_router_new(now_cb, NULL, NULL, 0);
  peer = seds_router_new(now_cb, NULL, NULL, 0);
  assert(g_router.r && peer);
  assert(seds_router_set_sender_id(g_router.r, "RF", 2) == SEDS_OK);
  assert(seds_router_set_sender_id(peer, "AB", 2) == SEDS_OK);
  assert(seds_router_add_side_packed(g_router.r, "CAN", 3, wire_tx, &to_peer, false) >= 0);
  assert(seds_router_add_side_packed(peer, "CAN", 3, wire_tx, &to_rf, false) >= 0);
  /* The Actuator Board's consumer configuration. */
  assert(seds_router_configure_timesync(peer, true, 0U, 100U, 5000U, 2000U, 2000U) == SEDS_OK);

  /* Boot without a fix: the production router configuration. */
  assert(telemetry_configure_timesync_locked(g_router.r) == SEDS_OK);
  assert(!telemetry_timesync_is_source());
  assert(telemetry_timesync_role() == TELEMETRY_TIMESYNC_ROLE_CONSUMER);
  run_ms(20000U, 0U, false);
  uint64_t utc = 0U;
  assert(rf_announces == 0U);
  assert(seds_router_get_network_time_ms(peer, &utc) != SEDS_OK);
  assert(seds_router_get_network_time_ms(g_router.r, &utc) != SEDS_OK);
  printf("NO_FIX announces=%u peer_time=none\n", rf_announces);

  /* A valid GPS date/time makes RF the priority-1 source with GPS UTC. */
  const uint64_t epoch = 1794183600000ULL; /* 2026-11-08T22:20:00Z */
  telemetry_set_unix_time_ms_locked(epoch + mono_ms);
  assert(telemetry_timesync_is_source() && telemetry_timesync_priority() == 1U);
  run_ms(10000U, epoch, true);
  assert(rf_announces >= 4U);
  assert(last_announce_time >= epoch && last_announce_time <= epoch + mono_ms);
  assert(seds_router_get_network_time_ms(peer, &utc) == SEDS_OK);
  assert(utc + 250U >= epoch + mono_ms && utc <= epoch + mono_ms + 250U);
  printf("FIX announces=%u peer_error_ms=%lld\n", rf_announces,
         (long long)utc - (long long)(epoch + mono_ms));

  /* Fix lost: RF keeps leading through the bounded holdover only. */
  const unsigned at_loss = rf_announces;
  run_ms(TELEMETRY_TIMESYNC_GPS_HOLDOVER_MS - 2000U, epoch, false);
  assert(telemetry_timesync_is_source() && rf_announces > at_loss);
  run_ms(3000U, epoch, false);
  assert(!telemetry_timesync_is_source() && !g_local_unix_valid);
  const unsigned after_expiry = rf_announces;
  run_ms(20000U, epoch, false);
  assert(rf_announces == after_expiry);
  printf("HOLDOVER announces_after_expiry=%u\n", rf_announces - after_expiry);

  /* A new fix restores the source role at once. */
  telemetry_set_unix_time_ms_locked(epoch + mono_ms);
  assert(telemetry_timesync_is_source());
  run_ms(4000U, epoch, true);
  assert(rf_announces > after_expiry);
  assert(last_announce_time >= epoch);
  seds_router_free(peer);
  seds_router_free(g_router.r);
  return 0;
}
