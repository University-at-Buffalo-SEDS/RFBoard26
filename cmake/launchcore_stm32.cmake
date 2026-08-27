include(FetchContent)
if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
endif()
find_package(Python3 COMPONENTS Interpreter REQUIRED)

foreach(required_var IN ITEMS
        LAUNCHCORE_DEVICE_DEFINE LAUNCHCORE_CMSIS_DEVICE_DIR
        LAUNCHCORE_HAL_INCLUDE_DIR LAUNCHCORE_SYSTEM_SOURCE
        LAUNCHCORE_STARTUP_SOURCE LAUNCHCORE_APP_SLOT_BASE
        LAUNCHCORE_APP_VECTOR_TABLE LAUNCHCORE_METADATA0_BASE)
    if(NOT DEFINED ${required_var})
        message(FATAL_ERROR "${required_var} must be set before including launchcore_stm32.cmake")
    endif()
endforeach()

FetchContent_Declare(
    sedslaunchcore
    GIT_REPOSITORY https://github.com/University-at-Buffalo-SEDS/SEDSLaunchCore.git
    GIT_TAG 6e74648b14378936867827bc744c31a4b3cb49e8
    GIT_SHALLOW FALSE
)
FetchContent_GetProperties(sedslaunchcore)
if(NOT sedslaunchcore_POPULATED)
    FetchContent_Populate(sedslaunchcore)
endif()

if(DEFINED LAUNCHCORE_DELTA_SIZE)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        "${CMAKE_SOURCE_DIR}/Core/Src/ota_stream.c"
        "${CMAKE_SOURCE_DIR}/Core/Src/launchcore_delta_format.c"
        "${sedslaunchcore_SOURCE_DIR}/bootloader/src/crc32.c"
        "${sedslaunchcore_SOURCE_DIR}/bootloader/src/metadata.c"
        "${sedslaunchcore_SOURCE_DIR}/update_lib/src/delta_update.c"
        "${sedslaunchcore_SOURCE_DIR}/update_lib/src/confirm_boot.c"
        "${sedslaunchcore_SOURCE_DIR}/update_lib/src/update_status.c"
        "${CMAKE_SOURCE_DIR}/Bootloader/storage_dispatch.c"
        "${CMAKE_SOURCE_DIR}/Bootloader/storage_internal_flash.c")
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        "${CMAKE_SOURCE_DIR}/Bootloader"
        "${sedslaunchcore_SOURCE_DIR}/bootloader/include"
        "${sedslaunchcore_SOURCE_DIR}/update_lib/include")
endif()

set(_launchcore_core_sources
    app_install.c
    boot_select.c
    bootloader_update.c
    bundle.c
    delta.c
    crc32.c
    image_validate.c
    metadata.c
    persist.c
    recovery.c
    recovery_install.c
    sha256.c
)
list(TRANSFORM _launchcore_core_sources
     PREPEND "${sedslaunchcore_SOURCE_DIR}/bootloader/src/")

set(LAUNCHCORE_BOOTLOADER_TARGET "${CMAKE_PROJECT_NAME}Bootloader")
add_executable(${LAUNCHCORE_BOOTLOADER_TARGET}
    ${_launchcore_core_sources}
    "${sedslaunchcore_SOURCE_DIR}/bootloader/src/main.c"
    "${sedslaunchcore_SOURCE_DIR}/bootloader/src/boot_jump.c"
    "${CMAKE_SOURCE_DIR}/Bootloader/platform.c"
    "${CMAKE_SOURCE_DIR}/Bootloader/storage_dispatch.c"
    "${CMAKE_SOURCE_DIR}/Bootloader/storage_internal_flash.c"
    "${LAUNCHCORE_SYSTEM_SOURCE}"
    ${LAUNCHCORE_FLASH_SOURCES}
    "${LAUNCHCORE_STARTUP_SOURCE}"
)
target_include_directories(${LAUNCHCORE_BOOTLOADER_TARGET} PRIVATE
    "${CMAKE_SOURCE_DIR}/Bootloader"
    "${CMAKE_SOURCE_DIR}/Core/Inc"
    "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include"
    "${LAUNCHCORE_CMSIS_DEVICE_DIR}"
    "${LAUNCHCORE_HAL_INCLUDE_DIR}"
    "${sedslaunchcore_SOURCE_DIR}/bootloader/include"
)
target_compile_definitions(${LAUNCHCORE_BOOTLOADER_TARGET} PRIVATE
    USE_HAL_DRIVER
    ${LAUNCHCORE_DEVICE_DEFINE}
)
target_link_options(${LAUNCHCORE_BOOTLOADER_TARGET} PRIVATE
    -T "${CMAKE_SOURCE_DIR}/Bootloader/linker_bootloader.ld"
    -Wl,-Map=${LAUNCHCORE_BOOTLOADER_TARGET}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)
target_link_libraries(${LAUNCHCORE_BOOTLOADER_TARGET} m)

set(LAUNCHCORE_APP_VERSION "1.0.0" CACHE STRING
    "Version stored in the packaged LaunchCore application image")
set(_app_bin "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin")
set(_app_image "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.launchcore.img")
set(_bootloader_bin "${CMAKE_CURRENT_BINARY_DIR}/${LAUNCHCORE_BOOTLOADER_TARGET}.bin")
set(_factory_image "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.factory.bin")

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${_app_bin}"
    COMMAND "${Python3_EXECUTABLE}" "${sedslaunchcore_SOURCE_DIR}/tools/mkimage.py"
            --input "${_app_bin}"
            --output "${_app_image}"
            --slot-base "${LAUNCHCORE_APP_SLOT_BASE}"
            --vector-table "${LAUNCHCORE_APP_VECTOR_TABLE}"
            --version "${LAUNCHCORE_APP_VERSION}"
            --xip
    VERBATIM
)
add_custom_command(TARGET ${LAUNCHCORE_BOOTLOADER_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${LAUNCHCORE_BOOTLOADER_TARGET}>
            "${_bootloader_bin}"
    VERBATIM
)
add_custom_command(
    OUTPUT "${_factory_image}"
    COMMAND "${Python3_EXECUTABLE}" "${sedslaunchcore_SOURCE_DIR}/tools/mkfactory.py"
            --bootloader "${_bootloader_bin}"
            --bootloader-base 0x08000000
            --app "${_app_image}"
            --app-base "${LAUNCHCORE_APP_SLOT_BASE}"
            --metadata-base "${LAUNCHCORE_METADATA0_BASE}"
            --output "${_factory_image}"
    DEPENDS ${CMAKE_PROJECT_NAME} ${LAUNCHCORE_BOOTLOADER_TARGET}
    VERBATIM
)
add_custom_target(factory-image ALL DEPENDS "${_factory_image}")
