#ifndef PERSISTENT_STORE_H
#define PERSISTENT_STORE_H
#include "launchcore/persist.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
/* Caller owns the same physical-flash mutex through each OTA operation. */
bool persistent_store_enter(void);
bool persistent_store_leave(void);
bool persistent_store_controls_ready(void);
launchcore_persist_status_t persistent_store_boot_begin(void);
unsigned int persistent_store_runtime_start(void);
launchcore_persist_status_t persistent_store_init(void);
launchcore_persist_status_t persistent_store_get(uint32_t key, void *value, size_t *value_size);
launchcore_persist_status_t persistent_store_set(uint32_t key, const void *value, size_t value_size);
bool persistent_store_recovery_ready(void);
bool persistent_store_commission_pending(void);
void persistent_store_hold_commissioning(void);
void persistent_store_note_failure(void);
bool persistent_store_publish_commission_complete(void);
bool persistent_store_verify_commissioned_locked(uint64_t generation,const uint8_t floor[16],const uint8_t marker[40]);
#endif
