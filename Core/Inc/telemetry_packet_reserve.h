#ifndef RF_TELEMETRY_PACKET_RESERVE_H
#define RF_TELEMETRY_PACKET_RESERVE_H
#include "tx_api.h"
#include "telemetry_large_reserve.h"

/* Carve one contiguous arena before network initialization. Coalescing 256-byte
 * units replace the single 8 KiB lifeboat: schema serialization can hold an
 * input and output buffer simultaneously, including two 8 KiB buffers. */
#define RF_PACKET_RESERVE_BYTES TELEMETRY_RESERVE_BYTES
#define RF_PACKET_RESERVE_MIN TELEMETRY_RESERVE_MIN
#define RF_PACKET_RESERVE_STORAGE TELEMETRY_RESERVE_BYTES
static void *rf_packet_reserve_storage;
static UINT rf_packet_reserve_ready;
volatile UINT g_rf_packet_reserve_init_status;

static UINT rf_packet_reserve_init(TX_BYTE_POOL *backing)
{
    if (rf_packet_reserve_ready) return TX_SUCCESS;
    UINT status = tx_byte_allocate(backing, &rf_packet_reserve_storage,
                                  RF_PACKET_RESERVE_STORAGE, TX_NO_WAIT);
    if (status == TX_SUCCESS) large_reserve_init(rf_packet_reserve_storage);
    g_rf_packet_reserve_init_status = status;
    rf_packet_reserve_ready = status == TX_SUCCESS;
    return status;
}
static void *rf_packet_reserve_allocate(size_t size)
{
    return large_reserve_allocate(size);
}
static int rf_packet_reserve_owns(void *ptr)
{
    return large_reserve_owns(ptr);
}
static void rf_packet_reserve_release(void *ptr)
{
    large_reserve_release(ptr);
}
static ULONG rf_packet_reserve_available(void)
{
    return large_reserve_available();
}
#endif
