#pragma once

#include "sedsprintf.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --------------------------------------------------------------------------------------
// Build-time configuration
// --------------------------------------------------------------------------------------
//
// Define TELEMETRY_ENABLED to enable the telemetry subsystem.
// Define TELEMETRY_TIME_MASTER to 1 on the RF/GPS board (time master), 0 on clients.
//
#ifndef TELEMETRY_TIME_MASTER
#define TELEMETRY_TIME_MASTER 1
#endif

// --------------------------------------------------------------------------------------
// Router state
// --------------------------------------------------------------------------------------
typedef struct {
  SedsRouter *r;
  uint8_t created;
  uint64_t start_time;
} RouterState;

// A single global router state (defined in telemetry.c)
extern RouterState g_router;

// --------------------------------------------------------------------------------------
// Core telemetry plumbing
// --------------------------------------------------------------------------------------
SedsResult init_telemetry_router(void);

SedsResult tx_send(const uint8_t *bytes, size_t len, void *user);
void       rx_asynchronous(const uint8_t *bytes, size_t len);

SedsResult on_sd_packet(const SedsPacketView *pkt, void *user);

// Logging
SedsResult log_telemetry_synchronous(SedsDataType data_type,
                                     const void *data,
                                     size_t element_count,
                                     size_t element_size);

SedsResult log_telemetry_asynchronous(SedsDataType data_type,
                                      const void *data,
                                      size_t element_count,
                                      size_t element_size);

// Queue processing
SedsResult dispatch_tx_queue(void);
SedsResult process_rx_queue(void);

SedsResult dispatch_tx_queue_timeout(uint32_t timeout_ms);
SedsResult process_rx_queue_timeout(uint32_t timeout_ms);
SedsResult process_all_queues_timeout(uint32_t timeout_ms);

// Errors / debug
SedsResult print_telemetry_error(int32_t error_code);
SedsResult log_error_asynchronous(const char* fmt, ...);
SedsResult log_error_synchronous(const char* fmt, ...);

void die(const char *fmt, ...);

// --------------------------------------------------------------------------------------
// Time sync API
// --------------------------------------------------------------------------------------
//
// telemetry_now_ms():
//   - "network-synced monotonic" time in ms (same across boards once synced)
//
// telemetry_unix_ms():
//   - unix epoch ms (only valid once base is established)
//
// On the TIME MASTER (RF/GPS):
//   - Call telemetry_set_unix_time_ms(gps_unix_ms) whenever GPS time updates.
//   - Periodically call telemetry_timesync_announce(priority, telemetry_unix_ms()).
//   - Clients will periodically call telemetry_timesync_request().
//
uint64_t telemetry_now_ms(void);

uint64_t telemetry_unix_ms(void);
uint64_t telemetry_unix_s(void);
uint8_t  telemetry_unix_is_valid(void);

// Master / GPS thread calls this to set unix time base.
// (On clients it can be a no-op, depending on your telemetry.c implementation.)
void telemetry_set_unix_time_ms(uint64_t unix_ms);

// Clients periodically request a resync with the master.
// (On master this should be a no-op.)
SedsResult telemetry_timesync_request(void);

// Master periodically announces its unix time (and priority).
// NOTE: This is the signature that must match telemetry.c.
SedsResult telemetry_timesync_announce(uint64_t priority, uint64_t unix_ms);

// Convenience: announce using the current unix time helper.
// (If unix isn't valid yet, this will announce 0.)
static inline SedsResult telemetry_timesync_announce_auto(uint64_t priority) {
  return telemetry_timesync_announce(priority, telemetry_unix_ms());
}

#ifdef __cplusplus
} // extern "C"
#endif
