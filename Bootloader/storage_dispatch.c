#include "launchcore/storage.h"

/* Embedded builds must never link LaunchCore's 16 MiB host mock backing store. */
static const launchcore_storage_driver_t *active_driver;

void launchcore_storage_set_driver(const launchcore_storage_driver_t *driver)
{
    active_driver = driver;
}

const launchcore_storage_driver_t *launchcore_storage_driver(void)
{
    return active_driver;
}

launchcore_storage_status_t launchcore_storage_init(void)
{
    return active_driver != NULL ? active_driver->init() : LAUNCHCORE_STORAGE_ERR_INIT;
}

uint32_t launchcore_slot_base(launchcore_slot_id_t slot)
{
    const launchcore_storage_layout_t *layout = active_driver->layout();
    return slot == LAUNCHCORE_SLOT_A ? layout->slot_a_base : layout->slot_b_base;
}

uint32_t launchcore_slot_size(launchcore_slot_id_t slot)
{
    const launchcore_storage_layout_t *layout = active_driver->layout();
    return slot == LAUNCHCORE_SLOT_A ? layout->slot_a_size : layout->slot_b_size;
}

bool launchcore_storage_range_preserves_persistent(uint32_t addr, uint32_t len)
{
    const launchcore_storage_layout_t *layout = active_driver->layout();
    if (layout->persistent_data_size == 0u || len == 0u) return true;
    const uint64_t end = (uint64_t)addr + len;
    const uint64_t persistent_end =
        (uint64_t)layout->persistent_data_base + layout->persistent_data_size;
    return end <= layout->persistent_data_base || persistent_end <= addr;
}
