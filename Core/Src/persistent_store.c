#include "persistent_store.h"
#include "launchcore/storage.h"
#include "tx_api.h"
#include <stdbool.h>
extern const launchcore_storage_driver_t launchcore_board_storage_driver;
static TX_MUTEX mutex;
static UINT mutex_ready;
static void ensure_mutex(void) {
    if (mutex_ready == 0U && tx_thread_identify() != TX_NULL &&
        tx_mutex_create(&mutex, "persist", TX_INHERIT) == TX_SUCCESS) mutex_ready = 1U;
}
static bool lock_store(void) {
    ensure_mutex();
    return mutex_ready != 0U && tx_thread_identify() != TX_NULL &&
           tx_mutex_get(&mutex, TX_WAIT_FOREVER) == TX_SUCCESS;
}
static void unlock_store(bool locked) { if (locked) (void)tx_mutex_put(&mutex); }
launchcore_persist_status_t persistent_store_init(void) {
    launchcore_storage_set_driver(&launchcore_board_storage_driver);
    bool locked = lock_store();
    launchcore_persist_status_t status = launchcore_persist_init();
    unlock_store(locked);
    return status;
}
launchcore_persist_status_t persistent_store_get(uint32_t key, void *value, size_t *size) {
    bool locked = lock_store();
    launchcore_persist_status_t status = launchcore_persist_get(key, value, size);
    unlock_store(locked); return status;
}
launchcore_persist_status_t persistent_store_set(uint32_t key, const void *value, size_t size) {
    bool locked = lock_store();
    launchcore_persist_status_t status = launchcore_persist_set(key, value, size);
    unlock_store(locked); return status;
}
