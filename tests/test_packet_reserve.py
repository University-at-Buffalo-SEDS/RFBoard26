"""Compile the RF allocator and exercise its final packet reserve under heap failure."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class PacketReserveTests(unittest.TestCase):
    def test_fragmented_heap_fallback_exhaustion_and_reuse(self):
        source = (ROOT / "Core/Src/telemetry_hooks.c").read_text()
        allocator = source[source.index("void *telemetryMalloc("):source.index("void seds_error_msg(")]
        stub = r"""
#ifndef TEST_TX_API_H
#define TEST_TX_API_H
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef unsigned UINT;
typedef unsigned long ULONG;
typedef unsigned TX_BYTE_POOL;
typedef struct { void *storage; ULONG available; } TX_BLOCK_POOL;
#define TX_SUCCESS 0U
#define TX_NO_WAIT 0U
#define TX_NO_MEMORY 16U
#define TX_CALLER_ERROR 19U
#define TX_NULL NULL
#define TX_THREAD_GET_SYSTEM_STATE() 0U
static void *tx_thread_identify(void) { return NULL; }
static ULONG storage[1030];
static UINT heap_result = TX_NO_MEMORY;
static UINT create_result = TX_SUCCESS;
static unsigned initializing, ordinary_releases;
static TX_BYTE_POOL small, large, emergency;
static UINT tx_byte_allocate(TX_BYTE_POOL *pool, void **ptr, size_t size, UINT wait) {
    (void)pool; assert(wait == TX_NO_WAIT);
    if (initializing) { assert(size <= sizeof(storage)); *ptr=storage; return TX_SUCCESS; }
    *ptr=heap_result == TX_SUCCESS ? (void *)&small : NULL; return heap_result;
}
static UINT tx_byte_release(void *ptr) { assert(ptr == storage || ptr == &small); ordinary_releases++; return TX_SUCCESS; }
static UINT tx_byte_pool_info_get(TX_BYTE_POOL *pool, void *name, ULONG *available,
    ULONG *fragments, void *first, void *count, void *next) {
    (void)pool; (void)name; (void)first; (void)count; (void)next;
    *available=12000; *fragments=200; return TX_SUCCESS;
}
static UINT tx_block_pool_create(TX_BLOCK_POOL *pool, char *name, size_t size,
    void *memory, size_t bytes) {
    (void)name; assert(bytes == size+sizeof(void *));
    pool->storage=memory; pool->available=1; return create_result;
}
static UINT tx_block_allocate(TX_BLOCK_POOL *pool, void **ptr, UINT wait) {
    assert(wait == TX_NO_WAIT); if (!pool->available) return TX_NO_MEMORY;
    pool->available=0; *(TX_BLOCK_POOL **)pool->storage=pool;
    *ptr=(char *)pool->storage+sizeof(void *); return TX_SUCCESS;
}
static UINT tx_block_release(void *ptr) {
    TX_BLOCK_POOL *pool=*(TX_BLOCK_POOL **)((char *)ptr-sizeof(void *));
    assert(!pool->available); pool->available=1; return TX_SUCCESS;
}
static UINT tx_block_pool_info_get(TX_BLOCK_POOL *pool, void *name, ULONG *available,
    void *total, void *first, void *count, void *next) {
    (void)name; (void)total; (void)first; (void)count; (void)next;
    *available=pool->available; return TX_SUCCESS;
}
#endif
"""
        code = r"""
#include "telemetry_packet_reserve.h"
#define TELEMETRY_LARGE_ALLOCATION_THRESHOLD 1024U
static TX_BYTE_POOL *rust_byte_pool_external=&small;
static TX_BYTE_POOL *rust_large_byte_pool_external=&large;
static TX_BYTE_POOL *rust_emergency_byte_pool_external=&emergency;
static size_t g_telemetry_last_alloc_request, g_telemetry_max_alloc_request, g_telemetry_alloc_failure_request;
static ULONG g_telemetry_alloc_failure_available, g_telemetry_alloc_failure_fragments;
static UINT g_telemetry_alloc_failure_status;
static ULONG g_telemetry_alloc_failure_system_state;
static uint32_t g_telemetry_alloc_failure_thread;
static unsigned g_telemetry_alloc_cross_pool_recoveries, g_telemetry_alloc_reserve_recoveries,
    g_telemetry_alloc_reserve_rearms, g_telemetry_alloc_fail, g_telemetry_alloc_count, g_telemetry_free_count;
static void telemetry_memory_profile_sample(void) {}
""" + allocator + r"""
int main(void) {
    assert(!rf_packet_reserve_allocate(1024));
    initializing=1; create_result=TX_CALLER_ERROR;
    assert(rf_packet_reserve_init(&emergency)==TX_CALLER_ERROR);
    assert(ordinary_releases==1 && !rf_packet_reserve_ready);
    create_result=TX_SUCCESS;
    assert(rf_packet_reserve_init(&emergency)==TX_SUCCESS);
    initializing=0;
    assert(rf_packet_reserve_init(&emergency)==TX_SUCCESS);
    /* Even with 12K total free, fragmented byte pools reject the packet. */
    for (unsigned cycle=0; cycle<10000; cycle++) {
        assert(telemetryMalloc(32)==NULL); /* small objects cannot steal reserve */
        assert(telemetryMalloc(8193)==NULL);
        void *packet=telemetryMalloc(8192); assert(packet);
        assert(rf_packet_reserve_available()==0);
        memset(packet,0xa5,8192);
        assert(telemetryMalloc(1024)==NULL); /* one live owner */
        telemetryFree(packet);
        assert(rf_packet_reserve_available()==8192);
    }
    heap_result=TX_CALLER_ERROR;
    assert(telemetryMalloc(4096)==NULL);
    assert(g_telemetry_alloc_failure_status==TX_CALLER_ERROR);
    heap_result=TX_SUCCESS;
    void *normal=telemetryMalloc(4096); assert(normal==&small);
    assert(rf_packet_reserve_available()==8192); telemetryFree(normal);
    assert(g_telemetry_alloc_reserve_recoveries==10000);
    assert(g_telemetry_alloc_reserve_rearms==10000);
    assert(g_telemetry_alloc_count==g_telemetry_free_count);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)
            (path/"tx_api.h").write_text(stub)
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-fsanitize=address,undefined", "-I", tmp, "-I", str(ROOT/"Core/Inc"),
                "-x", "c", "-", "-o", str(path/"test")], input=code, text=True, check=True)
            subprocess.run([str(path/"test")], check=True)
