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
    GIT_TAG ef512087bc9d2104651dedb93626e6df256e4fb8
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

add_custom_command(
    OUTPUT "${sedsnet_SOURCE_DIR}/telemetry_config.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${SEDSNET_SCHEMA_FILE}" "${sedsnet_SOURCE_DIR}/telemetry_config.json"
    DEPENDS "${SEDSNET_SCHEMA_FILE}"
    VERBATIM
)
add_custom_target(board_sedsnet_schema
    DEPENDS "${sedsnet_SOURCE_DIR}/telemetry_config.json")
add_dependencies(sedsnet_build board_sedsnet_schema)
target_link_libraries(${CMAKE_PROJECT_NAME} sedsnet::sedsnet)
