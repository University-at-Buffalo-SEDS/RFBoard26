#ifndef PERSISTENT_STORE_H
#define PERSISTENT_STORE_H
#include "launchcore/persist.h"
#include <stddef.h>
#include <stdint.h>
launchcore_persist_status_t persistent_store_init(void);
launchcore_persist_status_t persistent_store_get(uint32_t key, void *value, size_t *value_size);
launchcore_persist_status_t persistent_store_set(uint32_t key, const void *value, size_t value_size);
#endif
