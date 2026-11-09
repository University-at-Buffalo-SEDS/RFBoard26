/* The operator ABORT crossing the RF relay, with the actual SDK.
 *
 * GS --radio-- RF --CAN-- {FC, PB}. RF and PB register exactly their
 * firmware endpoint handlers (BOARD_RECOVERY only) and RF hosts the same
 * managed variables as the firmware. The GS sends ABORT exactly as its
 * backend does (seven endpoints, no target senders, queued as local RX).
 * The run reports the GS's frozen receipt cohort, every end-to-end receipt
 * that reaches the GS, and how often the GS retransmits the ABORT.
 *
 * argv[2] selects the FC endpoint handler's result for ABORT: "ok" or
 * "error" (the latter is FC's on_fc_packet_impl default before its fix). */
#include "sedsnet.h"
#include "sedsnet_config.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define FC_BOARD_RECOVERY 2003U
#define PB_BOARD_RECOVERY 2004U

static uint64_t mono_ms = 1000U;
static uint64_t now_cb(void *user) { (void)user; return mono_ms; }

typedef struct { uint8_t bytes[8192]; size_t len; int from; } Frame;
typedef struct { Frame frames[256]; unsigned count; } Wire;
enum { GS, RF, FC, PB, NODES };
static SedsRouter *node[NODES];
static int32_t radio_side[2]; /* [0] GS's side to RF, [1] RF's side to GS */
static int32_t can_side[NODES];
static Wire to_gs, to_rf_radio, can_bus;
static unsigned gs_abort_tx, gs_receipts, fc_abort_calls, fc_endpoint_calls;
static unsigned pb_receipt_sends, fc_receipt_sends;
static bool fc_endpoint_fails;

static bool is_receipt(const uint8_t *bytes, size_t len, uint32_t *ty)
{
  SedsOwnedPacket *pkt = seds_pkt_unpack_owned(bytes, len);
  if (!pkt) return false;
  SedsPacketView v;
  bool receipt = false;
  if (seds_owned_pkt_view(pkt, &v) == SEDS_OK) {
    *ty = v.ty;
    receipt = v.sender_len >= 7U && memcmp(v.sender, "E2EACK:", 7U) == 0;
  }
  seds_owned_pkt_free(pkt);
  return receipt;
}
static void push(Wire *w, const uint8_t *bytes, size_t len, int from)
{
  assert(w->count < 256U && len <= sizeof w->frames[0].bytes);
  memcpy(w->frames[w->count].bytes, bytes, len);
  w->frames[w->count].len = len;
  w->frames[w->count++].from = from;
}
static SedsResult gs_radio_tx(const uint8_t *b, size_t n, void *u)
{
  (void)u; uint32_t ty = 0U;
  (void)is_receipt(b, n, &ty);
  if (ty == SEDS_DT_ABORT) ++gs_abort_tx;
  push(&to_rf_radio, b, n, GS);
  return SEDS_OK;
}
static SedsResult rf_radio_tx(const uint8_t *b, size_t n, void *u)
{
  (void)u; uint32_t ty = 0U;
  if (is_receipt(b, n, &ty)) ++gs_receipts;
  push(&to_gs, b, n, RF);
  return SEDS_OK;
}
static SedsResult can_tx(const uint8_t *b, size_t n, void *u)
{
  const int from = (int)(intptr_t)u; uint32_t ty = 0U;
  if (is_receipt(b, n, &ty)) {
    if (from == PB) ++pb_receipt_sends;
    if (from == FC) ++fc_receipt_sends;
  }
  push(&can_bus, b, n, from);
  return SEDS_OK;
}
static SedsResult ok_handler(const SedsPacketView *p, void *u) { (void)p; (void)u; return SEDS_OK; }
static SedsResult fc_abort(const SedsPacketView *p, void *u)
{
  (void)u;
  if (p->ty == SEDS_DT_ABORT) ++fc_abort_calls;
  return SEDS_OK;
}
static SedsResult fc_endpoint(const SedsPacketView *p, void *u)
{
  (void)u;
  if (p->ty != SEDS_DT_ABORT) return SEDS_OK;
  ++fc_endpoint_calls;
  return fc_endpoint_fails ? SEDS_HANDLER_ERROR : SEDS_OK;
}
static Wire w;
static void deliver(void)
{
  w = to_rf_radio; to_rf_radio.count = 0U;
  for (unsigned i = 0; i < w.count; i++)
    (void)seds_router_receive_packed_from_side(node[RF], (uint32_t)radio_side[1], w.frames[i].bytes, w.frames[i].len);
  w = to_gs; to_gs.count = 0U;
  for (unsigned i = 0; i < w.count; i++)
    (void)seds_router_receive_packed_from_side(node[GS], (uint32_t)radio_side[0], w.frames[i].bytes, w.frames[i].len);
  w = can_bus; can_bus.count = 0U;
  for (unsigned i = 0; i < w.count; i++)
    for (int n = RF; n < NODES; n++)
      if (n != w.frames[i].from)
        (void)seds_router_receive_packed_from_side(node[n], (uint32_t)can_side[n], w.frames[i].bytes, w.frames[i].len);
}
static void run_ms(uint64_t duration)
{
  for (uint64_t end = mono_ms + duration; mono_ms < end; mono_ms += 10U) {
    for (int n = 0; n < NODES; n++) {
      bool queued = false;
      (void)seds_router_poll_discovery(node[n], &queued);
      (void)seds_router_process_all_queues(node[n]);
    }
    deliver();
  }
}
static SedsRouter *make(const char *name, const SedsLocalEndpointDesc *locals, size_t n)
{
  SedsRouter *r = seds_router_new(now_cb, NULL, locals, n);
  assert(r);
  assert(seds_router_set_sender_id(r, name, strlen(name)) == SEDS_OK);
  if (strcmp(name, "GS") != 0) assert(seds_router_set_preferred_discovery_master(r, "GS", 2U) == SEDS_OK);
  return r;
}

int main(int argc, char **argv)
{
  setbuf(stdout, NULL);
  assert(argc == 3);
  assert(seds_schema_register_json_file(argv[1], strlen(argv[1])) == SEDS_OK);
  fc_endpoint_fails = strcmp(argv[2], "error") == 0;
  /* RF and GS use the same short radio retry cadence (RF telemetry.c). */
  SedsRuntimeTuningConfig tuning;
  assert(seds_get_runtime_tuning_config(&tuning) == SEDS_OK);
  tuning.reliable_retransmit_ms = 250U;
  tuning.reliable_max_retries = 17U;
  assert(seds_set_runtime_tuning_config(&tuning) == SEDS_OK);

  const SedsLocalEndpointDesc gs_locals[] = {{.endpoint = SEDS_EP_GROUND_STATION, .packet_handler = ok_handler}};
  const SedsLocalEndpointDesc rf_locals[] = {{.endpoint = SEDS_EP_BOARD_RECOVERY, .packet_handler = ok_handler}};
  const SedsLocalEndpointDesc fc_locals[] = {
      {.endpoint = SEDS_EP_ABORT, .packet_handler = fc_abort},
      {.endpoint = FC_BOARD_RECOVERY, .packet_handler = ok_handler},
      {.endpoint = SEDS_EP_FLIGHT_CONTROLLER, .packet_handler = fc_endpoint}};
  const SedsLocalEndpointDesc pb_locals[] = {{.endpoint = PB_BOARD_RECOVERY, .packet_handler = ok_handler}};
  node[GS] = make("GS", gs_locals, 1U);
  node[RF] = make("RF", rf_locals, 1U);
  node[FC] = make("FC", fc_locals, 3U);
  node[PB] = make("PB", pb_locals, 1U);
  /* Managed variables the firmware hosts (read-only on RF/PB). */
  const SedsDataType managed[] = {SEDS_DT_FLIGHT_STATE, SEDS_DT_AV_BAY_UNDERGLOW};
  for (int n = RF; n < NODES; n++)
    for (unsigned i = 0; i < 2U; i++) {
      assert(seds_router_enable_network_variable(node[n], managed[i], true, false) == SEDS_OK);
      assert(seds_router_on_network_variable_update(node[n], managed[i], ok_handler, NULL) == SEDS_OK);
    }
  radio_side[0] = seds_router_add_side_packed(node[GS], "radio", 5U, gs_radio_tx, NULL, false);
  radio_side[1] = seds_router_add_side_packed(node[RF], "radio", 5U, rf_radio_tx, NULL, false);
  for (int n = RF; n < NODES; n++)
    can_side[n] = seds_router_add_side_packed(node[n], "can", 3U, can_tx, (void *)(intptr_t)n, false);
  assert(radio_side[0] >= 0 && radio_side[1] >= 0 && can_side[RF] >= 0 && can_side[FC] >= 0 && can_side[PB] >= 0);

  run_ms(8000U);

  static const char reason[] = "Manual Abort Command Issued";
  const uint32_t eps[] = {SEDS_EP_GROUND_STATION, SEDS_EP_FLIGHT_CONTROLLER, SEDS_EP_VALVE_BOARD,
                          SEDS_EP_ACTUATOR_BOARD, SEDS_EP_ABORT, SEDS_EP_FLIGHT_STATE, SEDS_EP_SD_CARD};
  SedsPacketView abort_pkt = {.ty = SEDS_DT_ABORT, .data_size = sizeof reason - 1U, .sender = "GS", .sender_len = 2U,
                              .endpoints = eps, .num_endpoints = 7U, .timestamp = mono_ms,
                              .payload = (const uint8_t *)reason, .payload_len = sizeof reason - 1U};
  /* A tracked GS send: the station retransmits until its receipt cohort
   * acknowledges, as the bench sims show for the operator ABORT. */
  assert(seds_router_transmit_message_queue(node[GS], &abort_pkt) == SEDS_OK);
  (void)seds_router_process_all_queues(node[GS]);
  SedsReliablePendingRow rows[4];
  int32_t pending = seds_router_copy_reliable_pending(node[GS], 0U, rows, 4U);
  uint32_t cohort = 0U;
  for (int32_t i = 0; i < pending && i < 4; i++)
    if (rows[i].data_type == SEDS_DT_ABORT) cohort = rows[i].pending_count;
  printf("GS frozen ABORT receipt cohort=%u\n", cohort);

  run_ms(8000U);
  pending = seds_router_copy_reliable_pending(node[GS], 0U, rows, 4U);
  unsigned abort_pending = 0U;
  for (int32_t i = 0; i < pending && i < 4; i++)
    if (rows[i].data_type == SEDS_DT_ABORT) abort_pending = rows[i].pending_count ? rows[i].pending_count : 1U;
  printf("fc_endpoint=%s gs_abort_tx=%u gs_receipts=%u fc_receipts=%u pb_receipts=%u "
         "fc_abort_calls=%u fc_endpoint_calls=%u gs_abort_pending=%u\n",
         argv[2], gs_abort_tx, gs_receipts, fc_receipt_sends, pb_receipt_sends,
         fc_abort_calls, fc_endpoint_calls, abort_pending);
  for (int n = 0; n < NODES; n++) seds_router_free(node[n]);
  return 0;
}
