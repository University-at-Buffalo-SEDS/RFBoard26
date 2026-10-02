# Select main when reachable; preserve a usable local copy when disconnected.
# A source override is deliberately local and never fetched/reset.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(SEDSNET_REPOSITORY "https://github.com/Rylan-Meilutis/SEDSnet.git" CACHE STRING
    "SEDSnet repository (main branch)")
option(SEDSNET_OFFLINE "Use the existing local SEDSnet source without network access" OFF)
if(NOT FETCHCONTENT_SOURCE_DIR_SEDSNET)
    # Seed an empty cache entry so FetchContent cannot persist our automatic
    # normal-variable selection as a user override on the next configure.
    set(FETCHCONTENT_SOURCE_DIR_SEDSNET "" CACHE PATH "Explicit local SEDSnet source override")
    set(_sedsnet_offline_arg)
    if(SEDSNET_OFFLINE OR FETCHCONTENT_FULLY_DISCONNECTED OR
       FETCHCONTENT_UPDATES_DISCONNECTED OR FETCHCONTENT_UPDATES_DISCONNECTED_SEDSNET)
        set(_sedsnet_offline_arg --offline)
    endif()
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/sedsnet_source.py"
            --cache "${FETCHCONTENT_BASE_DIR}/sedsnet-main"
            --repository "${SEDSNET_REPOSITORY}"
            --fallback "${FETCHCONTENT_BASE_DIR}/sedsnet-src"
            --fallback "${SEDSNET_DIR}"
            --fallback "${CMAKE_SOURCE_DIR}/third_party/SEDSnet"
            --fallback "${CMAKE_SOURCE_DIR}/SEDSnet"
            ${_sedsnet_offline_arg}
        RESULT_VARIABLE _sedsnet_result
        OUTPUT_VARIABLE _sedsnet_source OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT _sedsnet_result EQUAL 0)
        message(FATAL_ERROR "Unable to select SEDSnet main or an on-disk fallback")
    endif()
    # Normal variable, not cached: retry main on the next configure.
    set(FETCHCONTENT_SOURCE_DIR_SEDSNET "${_sedsnet_source}")
endif()
set(SEDSNET_DIR "${FETCHCONTENT_SOURCE_DIR_SEDSNET}" CACHE PATH "Path to sedsnet crate root" FORCE)
# FetchContent skips PATCH_COMMAND for source overrides, so always prepare here.
execute_process(COMMAND "${CMAKE_COMMAND}"
    "-DSEDSNET_SOURCE_DIR=${SEDSNET_DIR}"
    "-DSEDSNET_SCHEMA_FILE=${SEDSNET_SCHEMA_FILE}"
    "-DSEDSNET_CRC32_DIR=${CMAKE_SOURCE_DIR}/third_party/embedded-crc32fast"
    -P "${CMAKE_CURRENT_LIST_DIR}/prepare_sedsnet.cmake"
    COMMAND_ERROR_IS_FATAL ANY)
