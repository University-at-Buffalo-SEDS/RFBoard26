if(NOT DEFINED SEDSNET_SOURCE_DIR OR NOT DEFINED SEDSNET_SCHEMA_FILE OR
   NOT DEFINED SEDSNET_CRC32_DIR)
    message(FATAL_ERROR
        "SEDSNET_SOURCE_DIR, SEDSNET_SCHEMA_FILE, and SEDSNET_CRC32_DIR are required")
endif()

# Source selection remains main with offline fallback, but an older fallback
# cannot decode the canonical IDs used by this firmware. Fail before building
# an image that would boot successfully while silently losing all application data.
file(READ "${SEDSNET_SOURCE_DIR}/C-Headers/sedsnet.h" _sedsnet_identity_header)
foreach(_identity_api IN ITEMS seds_endpoint_identity seds_dtype_identity)
    string(FIND "${_sedsnet_identity_header}" "${_identity_api}(" _identity_found)
    if(_identity_found EQUAL -1)
        message(FATAL_ERROR
            "SEDSnet source lacks canonical schema identities; refresh main online or provide a compatible on-disk source")
    endif()
endforeach()

file(COPY_FILE "${SEDSNET_SCHEMA_FILE}"
     "${SEDSNET_SOURCE_DIR}/telemetry_config.json" ONLY_IF_DIFFERENT)

# The upstream `embedded` convenience feature includes zstd and cryptography.
# This STM32G491 has 512 KiB of flash and uses CAN-FD, where compression is not
# required for compatibility. Keep v4's schema/discovery/timesync support while
# omitting those optional, large features so the application and bootloader fit.
set(_cargo_toml "${SEDSNET_SOURCE_DIR}/Cargo.toml")
file(READ "${_cargo_toml}" _cargo)
set(_upstream_embedded
    "embedded = [\"compression\", \"timesync\", \"discovery\", \"cryptography\", \"serde\", \"serde_json\", \"serde_json/alloc\"]")
set(_actuator_embedded
    "embedded = [\"timesync\", \"discovery\", \"serde\", \"serde_json\", \"serde_json/alloc\"]")
string(FIND "${_cargo}" "${_actuator_embedded}" _already_prepared)
if(_already_prepared EQUAL -1)
    string(FIND "${_cargo}" "${_upstream_embedded}" _upstream_found)
    if(_upstream_found EQUAL -1)
        message(FATAL_ERROR
            "SEDSNet embedded feature definition changed; review the flash-size preparation step")
    endif()
    string(REPLACE "${_upstream_embedded}" "${_actuator_embedded}" _cargo "${_cargo}")
    file(WRITE "${_cargo_toml}" "${_cargo}")
endif()

# crc32fast's default slicing-by-16 table consumes 16 KiB of flash. Use the
# board-owned, wire-compatible bitwise implementation on constrained targets.
file(TO_CMAKE_PATH "${SEDSNET_CRC32_DIR}" _crc32_path)

# A patch is only a version candidate: Cargo can prefer a newer registry CRC
# and silently bypass this board's small/hardware-backed implementation. Bind
# the SDK's direct dependency to the board path while preserving its version
# requirement. An incompatible SDK must fail rather than select another CRC.
string(REGEX MATCH "crc32fast = \\{ version = \"[^\"]+\", default-features = false(, path = \"[^\"]+\")? \\}" _crc32_dependency "${_cargo}")
if(NOT _crc32_dependency)
    message(FATAL_ERROR "SEDSnet crc32fast dependency changed; review the embedded CRC preparation")
endif()
string(REGEX REPLACE ", path = \"[^\"]+\"" "" _crc32_direct "${_crc32_dependency}")
string(REPLACE " }" ", path = \"${_crc32_path}\" }" _crc32_direct "${_crc32_direct}")
string(REPLACE "${_crc32_dependency}" "${_crc32_direct}" _cargo "${_cargo}")
# Repair an already prepared/moved checkout's old patch path as well.
string(REGEX REPLACE "crc32fast = \\{ path = \"[^\"]+\" \\}"
    "crc32fast = { path = \"${_crc32_path}\" }" _cargo "${_cargo}")
file(WRITE "${_cargo_toml}" "${_cargo}")
set(_crc32_patch
    "[patch.crates-io]\ncrc32fast = { path = \"${_crc32_path}\" }\n")
string(FIND "${_cargo}" "[patch.crates-io]" _crc32_already_patched)
if(_crc32_already_patched EQUAL -1)
    string(APPEND _cargo "\n${_crc32_patch}")
    file(WRITE "${_cargo_toml}" "${_cargo}")
endif()
