# Required application opt-in APIs; a reader-only/legacy fallback cannot compile
# the storage-owner policy. Version strings alone do not identify these APIs.
function(launchcore_require_strict_api source)
    if(NOT EXISTS "${source}/bootloader/include/launchcore/persist.h")
        message(FATAL_ERROR "LaunchCore strict journal header missing: ${source}")
    endif()
    include(CheckCSourceCompiles)
    set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
    set(CMAKE_REQUIRED_INCLUDES "${source}/bootloader/include")
    # Recheck on every source override/reconfigure; never reuse a stale success.
    unset(_launchcore_required_strict_api CACHE)
    check_c_source_compiles([=[
#include <stddef.h>
#include <stdint.h>
#include "launchcore/persist.h"
#include "launchcore/storage.h"
_Static_assert(LAUNCHCORE_PERSIST_MAX_VALUE_SIZE >= 40U, "marker record size");
_Static_assert(LAUNCHCORE_PERSIST_HEALTH_UNKNOWN==0 && LAUNCHCORE_PERSIST_HEALTH_VIRGIN==1 && LAUNCHCORE_PERSIST_HEALTH_HEALTHY==2 && LAUNCHCORE_PERSIST_HEALTH_CORRUPT==3 && LAUNCHCORE_PERSIST_HEALTH_AMBIGUOUS==4, "strict health semantics");
_Static_assert(LAUNCHCORE_PERSIST_RECORD_ABSENT==1 && LAUNCHCORE_PERSIST_RECORD_PRESENT==2 && LAUNCHCORE_PERSIST_RECORD_DELETED==3, "absent/present/tombstone distinction");
#define SIG(fn,type) _Static_assert(_Generic(&(fn), type:1, default:0), "wrong strict API signature")
SIG(launchcore_persist_probe_strict, launchcore_persist_status_t (*)(launchcore_persist_strict_info_t *));
SIG(launchcore_persist_get_strict, launchcore_persist_status_t (*)(uint32_t,void *,size_t *,launchcore_persist_strict_info_t *));
SIG(launchcore_persist_set_strict, launchcore_persist_status_t (*)(uint32_t,const void *,size_t,launchcore_persist_strict_info_t *));
SIG(launchcore_persist_delete_strict, launchcore_persist_status_t (*)(uint32_t,launchcore_persist_strict_info_t *));
SIG(launchcore_persist_provision_virgin_strict, launchcore_persist_status_t (*)(uint32_t,const void *,size_t,launchcore_persist_strict_info_t *));
int main(void) {
 launchcore_persist_strict_info_t info={0}; launchcore_storage_layout_t layout={0};
 info.health=LAUNCHCORE_PERSIST_HEALTH_HEALTHY; info.record=LAUNCHCORE_PERSIST_RECORD_ABSENT;
 info.bank_generation=1U;info.max_sequence=1U;layout.persistent_data_write_size=8U;
 return (int)(info.bank_generation+info.max_sequence+layout.persistent_data_write_size);
}
]=] _launchcore_required_strict_api)
    if(NOT _launchcore_required_strict_api)
        message(FATAL_ERROR "Selected LaunchCore lacks required strict probe/get/set/delete/virgin-provision APIs, record classification, or physical write granule. Use published e2eea7d94245168dc63d31e160f4cb7bdc7688c3 or a reviewed compatible source. Legacy/reader-only v1.0.0 is unsupported. Selected: ${source}")
    endif()
endfunction()
