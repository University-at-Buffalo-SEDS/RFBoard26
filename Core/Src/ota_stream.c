#include "ota_stream.h"

#include "board_config.h"
#include "launchcore/storage.h"
#include "launchcore/update.h"
#include "main.h"
#include "telemetry.h"
#include <stdbool.h>
#include <stddef.h>

#define OTA_OP_BEGIN_DELTA 0x01U
#define OTA_OP_CHUNK 0x02U
#define OTA_OP_FINISH 0x03U
#define OTA_OP_ABORT 0x04U
#define OTA_OP_STATUS 0x05U
#define OTA_RESPONSE_FLAG 0x80U
#define OTA_CONFIRM_DELAY_MS 5000ULL
#define OTA_RESET_DELAY_MS 250ULL

extern const launchcore_storage_driver_t launchcore_board_storage_driver;

typedef struct {
  SedsRouter *router;
  uint32_t stream_id;
  uint32_t expected_offset;
  uint32_t declared_size;
  uint64_t initialized_ms;
  uint64_t reset_requested_ms;
  bool active;
  bool connected;
  bool boot_confirmed;
  bool reset_requested;
} ota_stream_state_t;

static ota_stream_state_t state;

static uint32_t read_u32_le(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static void write_u32_le(uint8_t *p, uint32_t value)
{
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
  p[2] = (uint8_t)(value >> 16);
  p[3] = (uint8_t)(value >> 24);
}

static ota_stream_status_t map_status(launchcore_status_t status)
{
  switch (status) {
  case LAUNCHCORE_OK: return OTA_STREAM_OK;
  case LAUNCHCORE_ERR_NO_SPACE: return OTA_STREAM_NO_SPACE;
  case LAUNCHCORE_ERR_BAD_STATE: return OTA_STREAM_BAD_STATE;
  case LAUNCHCORE_ERR_BAD_IMAGE:
  case LAUNCHCORE_ERR_VERIFY: return OTA_STREAM_BAD_IMAGE;
  case LAUNCHCORE_ERR_FLASH:
  case LAUNCHCORE_ERR_METADATA: return OTA_STREAM_STORAGE_ERROR;
  default: return OTA_STREAM_INTERNAL_ERROR;
  }
}

static SedsResult send_response(uint8_t opcode, ota_stream_status_t status)
{
  uint8_t response[13];
  response[0] = (uint8_t)(opcode | OTA_RESPONSE_FLAG);
  write_u32_le(&response[1], (uint32_t)status);
  write_u32_le(&response[5], state.expected_offset);
  write_u32_le(&response[9], ota_stream_max_patch_size());
  return seds_router_send_p2p_stream(state.router, state.stream_id, response,
                                     sizeof(response));
}

static ota_stream_status_t handle_data(const uint8_t *payload, size_t len)
{
  if (payload == NULL || len == 0U) return OTA_STREAM_BAD_MESSAGE;
  const uint8_t opcode = payload[0];

  if (opcode == OTA_OP_BEGIN_DELTA) {
    if (len != 5U || state.active) return OTA_STREAM_BAD_STATE;
    const uint32_t size = read_u32_le(&payload[1]);
    launchcore_status_t result = launchcore_delta_update_begin(size);
    if (result == LAUNCHCORE_OK) {
      state.active = true;
      state.declared_size = size;
      state.expected_offset = 0U;
    }
    return map_status(result);
  }

  if (opcode == OTA_OP_CHUNK) {
    if (!state.active || len < 6U || len > 5U + OTA_STREAM_MAX_CHUNK)
      return OTA_STREAM_BAD_STATE;
    const uint32_t offset = read_u32_le(&payload[1]);
    const size_t chunk_len = len - 5U;
    if (offset != state.expected_offset) return OTA_STREAM_BAD_OFFSET;
    if (offset > state.declared_size || chunk_len > state.declared_size - offset)
      return OTA_STREAM_NO_SPACE;
    if ((offset % BOARD_FLASH_WRITE_ALIGNMENT) != 0U ||
        (offset + chunk_len < state.declared_size &&
         (chunk_len % BOARD_FLASH_WRITE_ALIGNMENT) != 0U))
      return OTA_STREAM_BAD_MESSAGE;
    launchcore_status_t result = launchcore_delta_update_write(&payload[5], chunk_len);
    if (result == LAUNCHCORE_OK) state.expected_offset += (uint32_t)chunk_len;
    return map_status(result);
  }

  if (opcode == OTA_OP_FINISH) {
    if (len != 1U || !state.active) return OTA_STREAM_BAD_STATE;
    launchcore_status_t result = launchcore_delta_update_finish();
    if (result == LAUNCHCORE_OK) {
      state.active = false;
      state.reset_requested = true;
      state.reset_requested_ms = telemetry_now_ms();
    }
    return map_status(result);
  }

  if (opcode == OTA_OP_ABORT) {
    if (len != 1U || !state.active) return OTA_STREAM_BAD_STATE;
    launchcore_status_t result = launchcore_delta_update_abort();
    if (result == LAUNCHCORE_OK) state.active = false;
    return map_status(result);
  }

  if (opcode == OTA_OP_STATUS && len == 1U) return OTA_STREAM_OK;
  return OTA_STREAM_BAD_MESSAGE;
}

static SedsResult ota_stream_event(const SedsP2pStreamEventView *event, void *user)
{
  (void)user;
  if (event == NULL) return SEDS_BAD_ARG;
  if (event->kind == SEDS_P2P_STREAM_ACCEPTED) {
    if (state.connected && state.stream_id != event->stream_id) return SEDS_ERR;
    state.stream_id = event->stream_id;
    state.connected = true;
    return SEDS_OK;
  }
  if (event->stream_id != state.stream_id) return SEDS_ERR;
  if (event->kind == SEDS_P2P_STREAM_DATA) {
    const uint8_t opcode = event->payload_len != 0U ? event->payload[0] : 0U;
    return send_response(opcode, handle_data(event->payload, event->payload_len));
  }
  if (event->kind == SEDS_P2P_STREAM_CLOSED ||
      event->kind == SEDS_P2P_STREAM_RESET) {
    if (state.active) (void)launchcore_delta_update_abort();
    state.active = false;
    state.connected = false;
    state.stream_id = 0U;
  }
  return SEDS_OK;
}

uint32_t ota_stream_max_patch_size(void) { return BOARD_DELTA_SIZE; }

SedsResult ota_stream_init(SedsRouter *router)
{
  if (router == NULL) return SEDS_BAD_ARG;
  launchcore_storage_set_driver(&launchcore_board_storage_driver);
  if (launchcore_storage_init() != LAUNCHCORE_STORAGE_OK) return SEDS_ERR;
  state = (ota_stream_state_t){.router = router, .initialized_ms = telemetry_now_ms()};
  return seds_router_bind_p2p_stream_port(router, OTA_STREAM_PORT,
                                          ota_stream_event, NULL);
}

void ota_stream_poll(void)
{
  const uint64_t now = telemetry_now_ms();
  if (!state.boot_confirmed && now - state.initialized_ms >= OTA_CONFIRM_DELAY_MS)
    state.boot_confirmed = launchcore_confirm_boot() == LAUNCHCORE_OK;
  if (state.reset_requested && now - state.reset_requested_ms >= OTA_RESET_DELAY_MS)
    NVIC_SystemReset();
}

