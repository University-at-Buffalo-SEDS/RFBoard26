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
#define TX_SUCCESS 0U
#define TX_NO_WAIT 0U
#define TX_NO_MEMORY 16U
#define TX_CALLER_ERROR 19U
#define TX_NULL NULL
#define TX_THREAD_GET_SYSTEM_STATE() 0U
static void *tx_thread_identify(void) { return NULL; }
static ULONG storage[4096];
static uint32_t mask;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t m){mask=m;}
static UINT heap_result = TX_NO_MEMORY;
static UINT init_result = TX_SUCCESS;
static unsigned initializing, ordinary_releases;
static TX_BYTE_POOL small, large, emergency;
static UINT tx_byte_allocate(TX_BYTE_POOL *pool, void **ptr, size_t size, UINT wait) {
    (void)pool; assert(wait == TX_NO_WAIT);
    if (initializing) { assert(size <= sizeof(storage)); *ptr=storage; return init_result; }
    *ptr=heap_result == TX_SUCCESS ? (void *)&small : NULL; return heap_result;
}
static UINT tx_byte_release(void *ptr) { assert(ptr == storage || ptr == &small); ordinary_releases++; return TX_SUCCESS; }
static UINT tx_byte_pool_info_get(TX_BYTE_POOL *pool, void *name, ULONG *available,
    ULONG *fragments, void *first, void *count, void *next) {
    (void)pool; (void)name; (void)first; (void)count; (void)next;
    *available=12000; *fragments=200; return TX_SUCCESS;
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
    assert(!rf_packet_reserve_allocate(4096));
    initializing=1; init_result=TX_NO_MEMORY;
    assert(rf_packet_reserve_init(&emergency)==TX_NO_MEMORY && !rf_packet_reserve_ready);
    init_result=TX_SUCCESS;
    assert(rf_packet_reserve_init(&emergency)==TX_SUCCESS);
    initializing=0;
    for (unsigned cycle=0; cycle<10000; cycle++) {
        assert(telemetryMalloc(32)==NULL);
        assert(telemetryMalloc(16385)==NULL);
        void *a=telemetryMalloc(8192),*b=telemetryMalloc(8192);
        assert(a && b && a!=b && !telemetryMalloc(4096));
        assert(rf_packet_reserve_available()==0);
        assert(rf_packet_reserve_init(&emergency)==TX_SUCCESS);
        assert(rf_packet_reserve_available()==0); // repeated init preserves ownership
        telemetryFree(a);telemetryFree(b);
        a=telemetryMalloc(4112);b=telemetryMalloc(3640);void*c=telemetryMalloc(3640);
        assert(a&&b&&c);telemetryFree(a);telemetryFree(b);telemetryFree(c);
        assert(rf_packet_reserve_available()==16384);
    }
    heap_result=TX_SUCCESS;void*ordinary=telemetryMalloc(32);assert(ordinary==&small);
    telemetryFree(ordinary);assert(ordinary_releases==1);
    assert(g_telemetry_alloc_count==g_telemetry_free_count && !mask);
}

"""
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)
            (path/"tx_api.h").write_text(stub)
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-fsanitize=address,undefined", "-I", tmp, "-I", str(ROOT/"Core/Inc"),
                "-x", "c", "-", "-o", str(path/"test")], input=code, text=True, check=True)
            subprocess.run([str(path/"test")], check=True)
