/* A refused FLIGHT_STATE write is still a delivered packet (actual SDK).
 *
 * The production flight_state_cache.c callback is registered on a real
 * router. A station write the recovery floor refuses (stale, store not
 * ready) and an out-of-range value are deterministic: retrying the same
 * packet can never succeed. They must be acknowledged, relayed onward and
 * never applied or persisted. A transient refusal (OTA owner busy) keeps
 * failing so that the station's retry is the recovery path. */
#include "flight_state_cache.h"
#include "board_recovery_transport.h"
#include "persistent_store.h"
#include "sedsnet_config.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

extern volatile uint32_t g_reliable_refusal_count;
extern volatile uint32_t g_flight_state_cache_writes;
volatile uint32_t g_telemetry_discovery_seen;
static uint64_t mono_ms = 1000U;
static int allowed = 1, ota_busy;
static unsigned writes, receipts, relayed;
static uint8_t stored = 1U;

uint32_t HAL_GetTick(void) { return (uint32_t)mono_ms; }
launchcore_persist_status_t persistent_store_init(void) { return LAUNCHCORE_PERSIST_OK; }
launchcore_persist_status_t persistent_store_get(uint32_t key, void *value, size_t *size)
{ (void)key; assert(*size == 1U); *(uint8_t *)value = stored; return LAUNCHCORE_PERSIST_OK; }
launchcore_persist_status_t persistent_store_set(uint32_t key, const void *value, size_t size)
{ (void)key; assert(size == 1U); stored = *(const uint8_t *)value; ++writes; return LAUNCHCORE_PERSIST_OK; }
uint8_t board_recovery_managed_update_allowed(const SedsPacketView *packet)
{ assert(packet); return (uint8_t)allowed; }
bool ota_stream_state_begin(uint8_t phase) { (void)phase; return !ota_busy; }
void ota_stream_state_end(void) {}

static uint64_t now_cb(void *user) { (void)user; return mono_ms; }
static SedsResult ok_handler(const SedsPacketView *p, void *u) { (void)p; (void)u; return SEDS_OK; }
static SedsResult upstream_tx(const SedsPacketView *p, void *u)
{
  (void)u;
  if (p->sender_len >= 7U && memcmp(p->sender, "E2EACK:", 7U) == 0) ++receipts;
  return SEDS_OK;
}
static SedsResult downstream_tx(const SedsPacketView *p, void *u)
{
  (void)u;
  if (p->ty == SEDS_DT_FLIGHT_STATE) ++relayed;
  return SEDS_OK;
}
typedef struct { SedsRouter *board; uint32_t side; } Target;
static SedsResult learn(const SedsPacketView *p, void *u)
{
  const Target *t = (const Target *)u;
  return seds_router_receive_from_side(t->board, t->side, p);
}

static SedsRouter *board;
static int32_t upstream, downstream;
static SedsResult deliver(const char *sender, uint8_t value)
{
  const uint32_t eps[] = {SEDS_EP_FLIGHT_STATE, SEDS_EP_SD_CARD, SEDS_EP_GROUND_STATION};
  mono_ms += 100U;
  SedsPacketView p = {.ty = SEDS_DT_FLIGHT_STATE, .data_size = 1U, .sender = sender,
                      .sender_len = strlen(sender), .endpoints = eps, .num_endpoints = 3U,
                      .timestamp = mono_ms, .payload = &value, .payload_len = 1U};
  const SedsResult result = seds_router_receive_from_side(board, (uint32_t)upstream, &p);
  (void)seds_router_process_all_queues(board);
  return result;
}

int main(int argc, char **argv)
{
  setbuf(stdout, NULL);
  assert(argc == 2);
  assert(seds_schema_register_json_file(argv[1], strlen(argv[1])) == SEDS_OK);
  const SedsLocalEndpointDesc local = {.endpoint = SEDS_EP_BOARD_RECOVERY, .packet_handler = ok_handler};
  board = seds_router_new(now_cb, NULL, &local, 1U);
  assert(board && seds_router_set_sender_id(board, "RF", 2U) == SEDS_OK);
  upstream = seds_router_add_side_packet(board, "radio", 5U, upstream_tx, NULL, false);
  downstream = seds_router_add_side_packet(board, "can", 3U, downstream_tx, NULL, false);
  assert(upstream >= 0 && downstream >= 0);
  assert(flight_state_cache_init(board) == SEDS_OK);
  board_recovery_transport_set_router(board);
  /* The board learns the station through discovery, as on the bench. */
  const SedsLocalEndpointDesc gs_local = {.endpoint = SEDS_EP_GROUND_STATION, .packet_handler = ok_handler};
  SedsRouter *gs = seds_router_new(now_cb, NULL, &gs_local, 1U);
  assert(gs && seds_router_set_sender_id(gs, "GS", 2U) == SEDS_OK);
  Target target = {board, (uint32_t)upstream};
  assert(seds_router_add_side_packet(gs, "radio", 5U, learn, &target, false) >= 0);
  bool queued = false;
  assert(seds_router_poll_discovery(gs, &queued) == SEDS_OK);
  assert(seds_router_process_all_queues(gs) == SEDS_OK);
  assert(seds_router_process_all_queues(board) == SEDS_OK);
  seds_router_free(gs);
  receipts = relayed = 0U;

  /* Accepted write: applied, persisted, acknowledged and relayed. */
  assert(deliver("GS", 3U) == SEDS_OK);
  assert(flight_state_cache_value() == 3U && writes == 1U && receipts == 1U && relayed == 1U);
  const uint32_t refusals = g_reliable_refusal_count;

  /* Station write refused by the recovery floor: acknowledged, never applied. */
  allowed = 0;
  SedsResult r = deliver("GS", 7U);
  printf("refused result=%d receipts=%u relayed=%u value=%u writes=%u refusals=%u\n", r, receipts,
         relayed, flight_state_cache_value(), writes, (unsigned)(g_reliable_refusal_count - refusals));
  assert(r == SEDS_OK && receipts == 2U && relayed == 2U);
  assert(flight_state_cache_value() == 3U && writes == 1U && stored == 3U);
  /* The SDK may re-run the callback on relay of a retired observation. */
  assert(g_reliable_refusal_count > refusals);
  const uint32_t after_refused = g_reliable_refusal_count;

  /* Out-of-range value: acknowledged, never applied. */
  allowed = 1;
  r = deliver("GS", 16U);
  printf("malformed result=%d receipts=%u value=%u writes=%u\n", r, receipts, flight_state_cache_value(), writes);
  assert(r == SEDS_OK && receipts == 3U && flight_state_cache_value() == 3U && writes == 1U);
  assert(g_reliable_refusal_count > after_refused);

  /* Transient: the OTA owner is busy. No receipt, so the station retries. */
  ota_busy = 1;
  r = deliver("GS", 4U);
  printf("transient result=%d receipts=%u value=%u\n", r, receipts, flight_state_cache_value());
  assert(r != SEDS_OK && receipts == 3U && flight_state_cache_value() == 3U);
  ota_busy = 0;
  assert(deliver("GS", 4U) == SEDS_OK && receipts == 4U && flight_state_cache_value() == 4U && writes == 2U);

  seds_router_free(board);
  puts("PASS refused and malformed FLIGHT_STATE acknowledged and relayed without applying; transient refusal retried");
  return 0;
}
