include(FetchContent)

set(SEDSNET_SCHEMA_FILE "${CMAKE_SOURCE_DIR}/config/sedsnet.json")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             "${SEDSNET_SCHEMA_FILE}")

set(SEDSNET_FORCE_RELEASE ON CACHE BOOL
    "Build SEDSNet in release mode for embedded firmware" FORCE)
set(SEDSNET_EMBEDDED_BUILD ON CACHE BOOL "Build SEDSNet for an embedded target" FORCE)
set(SEDSNET_ENABLE_CRYPTOGRAPHY OFF CACHE BOOL
    "Keep the current unencrypted embedded transport" FORCE)

FetchContent_Declare(
    sedsnet
    GIT_REPOSITORY https://github.com/Rylan-Meilutis/SEDSnet.git
    GIT_TAG 3b2eb63edc17aa8feb14b1f90d6bb91cbb417752
    GIT_SHALLOW FALSE
    PATCH_COMMAND ${CMAKE_COMMAND}
                  -DSEDSNET_SOURCE_DIR=<SOURCE_DIR>
                  -DSEDSNET_SCHEMA_FILE=${SEDSNET_SCHEMA_FILE}
                  -DSEDSNET_CRC32_DIR=${CMAKE_SOURCE_DIR}/third_party/embedded-crc32fast
                  -P ${CMAKE_SOURCE_DIR}/cmake/prepare_sedsnet.cmake
)
FetchContent_MakeAvailable(sedsnet)
# SEDSNet's assembler-stack-note flag is not consumed by GCC's LTO wrapper.
# The final ELF still enforces a non-executable stack through -z noexecstack.
set_property(TARGET sedsnet PROPERTY INTERFACE_COMPILE_OPTIONS "")

# Copy the board schema during configure, before Ninja calculates whether the
# Rust archive is stale. Deleting/touching files from a build-time dependency
# races Ninja's initial dirty check and can remove the archive immediately
# before the firmware link step.
configure_file("${SEDSNET_SCHEMA_FILE}"
               "${sedsnet_SOURCE_DIR}/telemetry_config.json" COPYONLY)
file(TOUCH "${sedsnet_SOURCE_DIR}/build.rs")
target_link_libraries(${CMAKE_PROJECT_NAME} sedsnet::sedsnet)
add_dependencies(${CMAKE_PROJECT_NAME} sedsnet_build)
