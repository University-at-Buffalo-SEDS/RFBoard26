#include "board_config.h"
#include "launchcore/storage.h"
#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <string.h>

static const launchcore_storage_layout_t layout = {
    .slot_a_base = BOARD_SLOT_A_BASE,
    .slot_a_size = BOARD_SLOT_A_SIZE,
    .slot_b_base = BOARD_DELTA_BASE,
    .slot_b_size = BOARD_DELTA_SIZE,
    .metadata0_base = BOARD_METADATA0_BASE,
    .metadata1_base = BOARD_METADATA1_BASE,
    .metadata_size = LAUNCHCORE_FLASH_ERASE_SIZE,
    .bootloader_base = LAUNCHCORE_INTERNAL_FLASH_BASE,
    .bootloader_size = LAUNCHCORE_BOOTLOADER_SIZE,
    .persistent_data_base = BOARD_PERSIST_BASE,
    .persistent_data_size = BOARD_PERSIST_SIZE,
    .persistent_data_erase_size = LAUNCHCORE_FLASH_ERASE_SIZE,
    .slot_erase_size = LAUNCHCORE_FLASH_ERASE_SIZE,
    .slot_b_is_delta = true,
    .supports_xip = true,
};

static bool range_within(uint32_t addr, uint32_t len, uint32_t base, uint32_t size)
{
    return len <= size && addr >= base && addr - base <= size - len;
}

static bool writable_range(uint32_t addr, uint32_t len)
{
    return range_within(addr, len, layout.slot_a_base, layout.slot_a_size) ||
           range_within(addr, len, layout.slot_b_base, layout.slot_b_size) ||
           range_within(addr, len, layout.metadata0_base, layout.metadata_size) ||
           range_within(addr, len, layout.metadata1_base, layout.metadata_size) ||
           range_within(addr, len, layout.persistent_data_base, layout.persistent_data_size);
}

static launchcore_storage_status_t storage_init(void) { return LAUNCHCORE_STORAGE_OK; }

static void flash_page_for_address(uint32_t addr, uint32_t *bank, uint32_t *page)
{
#if defined(FLASH_OPTR_DBANK)
    if ((FLASH->OPTR & FLASH_OPTR_DBANK) != 0u &&
        addr >= FLASH_BASE + FLASH_BANK_SIZE)
    {
        *bank = FLASH_BANK_2;
        *page = (addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
        return;
    }
#endif
    *bank = FLASH_BANK_1;
    *page = (addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}

static launchcore_storage_status_t storage_erase(uint32_t addr, uint32_t len)
{
    if (len == 0u || (addr % FLASH_PAGE_SIZE) != 0u || (len % FLASH_PAGE_SIZE) != 0u ||
        !writable_range(addr, len))
    {
        return LAUNCHCORE_STORAGE_ERR_RANGE;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return LAUNCHCORE_STORAGE_ERR_ERASE;
    }

    launchcore_storage_status_t status = LAUNCHCORE_STORAGE_OK;
    for (uint32_t current = addr; current < addr + len; current += FLASH_PAGE_SIZE)
    {
        FLASH_EraseInitTypeDef erase = {0};
        uint32_t page_error = 0u;
        erase.TypeErase = FLASH_TYPEERASE_PAGES;
        erase.NbPages = 1u;
        flash_page_for_address(current, &erase.Banks, &erase.Page);
        if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
        {
            status = LAUNCHCORE_STORAGE_ERR_ERASE;
            break;
        }
    }
    (void)HAL_FLASH_Lock();
    return status;
}

static launchcore_storage_status_t storage_write(uint32_t addr, const void *data, uint32_t len)
{
    if (data == NULL || len == 0u || (addr & 7u) != 0u || !writable_range(addr, len))
    {
        return LAUNCHCORE_STORAGE_ERR_RANGE;
    }
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return LAUNCHCORE_STORAGE_ERR_WRITE;
    }

    const uint8_t *src = (const uint8_t *)data;
    launchcore_storage_status_t status = LAUNCHCORE_STORAGE_OK;
    for (uint32_t offset = 0u; offset < len; offset += 8u)
    {
        uint8_t bytes[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};
        uint64_t word;
        uint32_t take = len - offset;
        if (take > sizeof(bytes)) take = sizeof(bytes);
        memcpy(bytes, src + offset, take);
        memcpy(&word, bytes, sizeof(word));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + offset, word) != HAL_OK)
        {
            status = LAUNCHCORE_STORAGE_ERR_WRITE;
            break;
        }
    }
    (void)HAL_FLASH_Lock();
    if (status == LAUNCHCORE_STORAGE_OK &&
        memcmp((const void *)(uintptr_t)addr, data, len) != 0)
    {
        status = LAUNCHCORE_STORAGE_ERR_VERIFY;
    }
    return status;
}

static launchcore_storage_status_t storage_read(uint32_t addr, void *data, uint32_t len)
{
    if (data == NULL ||
        !range_within(addr, len, LAUNCHCORE_INTERNAL_FLASH_BASE,
                      LAUNCHCORE_INTERNAL_FLASH_SIZE))
    {
        return LAUNCHCORE_STORAGE_ERR_RANGE;
    }
    memcpy(data, (const void *)(uintptr_t)addr, len);
    return LAUNCHCORE_STORAGE_OK;
}

static launchcore_storage_status_t enable_memory_mapped(void)
{
    return LAUNCHCORE_STORAGE_OK;
}

static bool executable(uint32_t addr)
{
    return addr >= BOARD_VECTOR_TABLE &&
           addr < BOARD_SLOT_A_BASE + BOARD_SLOT_A_SIZE;
}

static const launchcore_storage_layout_t *layout_fn(void) { return &layout; }

const launchcore_storage_driver_t launchcore_board_storage_driver = {
    .init = storage_init,
    .erase = storage_erase,
    .write = storage_write,
    .read = storage_read,
    .enable_memory_mapped = enable_memory_mapped,
    .is_executable_addr = executable,
    .layout = layout_fn,
};
