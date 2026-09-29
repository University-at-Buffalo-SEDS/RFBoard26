#ifndef RF_TELEMETRY_PACKET_RESERVE_H
#define RF_TELEMETRY_PACKET_RESERVE_H
#include "tx_api.h"
#include <stddef.h>
#include <stdint.h>

/* Carved from the existing emergency heap before network initialization.
 * Small routing records cannot consume or fragment this final packet block. */
#define RF_PACKET_RESERVE_BYTES 8192U
#define RF_PACKET_RESERVE_MIN 1024U
#define RF_PACKET_RESERVE_STORAGE (RF_PACKET_RESERVE_BYTES + sizeof(void *))
static TX_BLOCK_POOL rf_packet_reserve_pool;
static void *rf_packet_reserve_storage;
static UINT rf_packet_reserve_ready;
volatile UINT g_rf_packet_reserve_init_status;

static UINT rf_packet_reserve_init(TX_BYTE_POOL *backing)
{
    if (rf_packet_reserve_ready) return TX_SUCCESS;
    UINT status = tx_byte_allocate(backing, &rf_packet_reserve_storage,
                                  RF_PACKET_RESERVE_STORAGE, TX_NO_WAIT);
    if (status == TX_SUCCESS) {
        status = tx_block_pool_create(&rf_packet_reserve_pool, "RF packet reserve",
            RF_PACKET_RESERVE_BYTES, rf_packet_reserve_storage,
            RF_PACKET_RESERVE_STORAGE);
        if (status != TX_SUCCESS) {
            (void)tx_byte_release(rf_packet_reserve_storage);
            rf_packet_reserve_storage = NULL;
        }
    }
    g_rf_packet_reserve_init_status = status;
    rf_packet_reserve_ready = status == TX_SUCCESS;
    return status;
}

static void *rf_packet_reserve_allocate(size_t size)
{
    void *ptr = NULL;
    if (!rf_packet_reserve_ready || size < RF_PACKET_RESERVE_MIN ||
        size > RF_PACKET_RESERVE_BYTES) return NULL;
    if (tx_block_allocate(&rf_packet_reserve_pool, &ptr, TX_NO_WAIT) != TX_SUCCESS)
        return NULL;
    return ptr;
}

static int rf_packet_reserve_owns(void *ptr)
{
    const uintptr_t address = (uintptr_t)ptr;
    const uintptr_t start = (uintptr_t)rf_packet_reserve_storage;
    return rf_packet_reserve_ready && address >= start &&
           address < start + RF_PACKET_RESERVE_STORAGE;
}

static ULONG rf_packet_reserve_available(void)
{
    ULONG available = 0U;
    if (rf_packet_reserve_ready)
        (void)tx_block_pool_info_get(&rf_packet_reserve_pool, TX_NULL, &available,
                                    TX_NULL, TX_NULL, TX_NULL, TX_NULL);
    return available * RF_PACKET_RESERVE_BYTES;
}
#endif
