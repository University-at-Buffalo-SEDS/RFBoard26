#pragma once

#include "sedsnet_config.h"
#include <stdint.h>

#define OTA_STREAM_PORT 4510U
#define OTA_STREAM_MAX_CHUNK 112U

typedef enum {
  OTA_STREAM_OK = 0,
  OTA_STREAM_BAD_MESSAGE = 1,
  OTA_STREAM_BAD_STATE = 2,
  OTA_STREAM_BAD_OFFSET = 3,
  OTA_STREAM_NO_SPACE = 4,
  OTA_STREAM_STORAGE_ERROR = 5,
  OTA_STREAM_BAD_IMAGE = 6,
  OTA_STREAM_INTERNAL_ERROR = 7,
} ota_stream_status_t;

SedsResult ota_stream_init(SedsRouter *router);
void ota_stream_poll(void);
uint32_t ota_stream_max_patch_size(void);

