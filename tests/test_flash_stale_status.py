"""The storage driver must not fail on FLASH_SR bits left before its operation.

A DFU ':leave' starts the image without a reset, so the ROM loader's sticky
FLASH_SR error bits (or OPTVERR from reset) are still set when the first boot
provisions the persistent store. HAL reports any such bit as a failure of the
next erase/program, which faulted the store owner and with it the OTA stream.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

HAL = r'''
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct { uint32_t TypeErase, Banks, Page, NbPages; } FLASH_EraseInitTypeDef;
typedef struct { uint32_t SR, CR; } FLASH_TypeDef;
extern FLASH_TypeDef fake_flash;
#define FLASH (&fake_flash)
#define FLASH_BASE 0x08000000UL
#define FLASH_PAGE_SIZE 0x800U
#define FLASH_BANK_1 1U
#define FLASH_TYPEERASE_PAGES 0U
#define FLASH_TYPEPROGRAM_DOUBLEWORD 0U
#define FLASH_FLAG_OPTVERR (1UL << 15)
#define FLASH_FLAG_PGSERR (1UL << 7)
#define FLASH_FLAG_PROGERR (1UL << 3)
#define FLASH_FLAG_SR_ERRORS (0xC3FAUL)
#define __HAL_FLASH_CLEAR_FLAG(f) (FLASH->SR &= ~(uint32_t)(f))
#define __HAL_FLASH_INSTRUCTION_CACHE_RESET() ((void)0)
/* As stm32g4xx_hal_flash.c: FLASH_WaitForLastOperation() fails, and clears,
 * on any error bit that is set when an operation starts. */
static inline HAL_StatusTypeDef wait_last(void)
{
    uint32_t error = FLASH->SR & FLASH_FLAG_SR_ERRORS;
    if (error) { __HAL_FLASH_CLEAR_FLAG(error); return HAL_ERROR; }
    return HAL_OK;
}
static inline HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return HAL_OK; }
static inline HAL_StatusTypeDef HAL_FLASH_Lock(void) { return HAL_OK; }
static inline HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e, uint32_t *bad)
{
    *bad = 0xFFFFFFFFU;
    if (wait_last() != HAL_OK) return HAL_ERROR;
    memset((void *)(uintptr_t)(FLASH_BASE + e->Page * FLASH_PAGE_SIZE), 0xFF, FLASH_PAGE_SIZE);
    return HAL_OK;
}
static inline HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t addr, uint64_t data)
{
    (void)type;
    if (wait_last() != HAL_OK) return HAL_ERROR;
    uint64_t *cell = (uint64_t *)(uintptr_t)addr;
    if (*cell != UINT64_MAX) { FLASH->SR |= FLASH_FLAG_PROGERR; return HAL_ERROR; }
    *cell = data;
    return HAL_OK;
}
'''

MAIN = r'''
#include <assert.h>
#include <sys/mman.h>
#include "launchcore/storage.h"
#include "board_config.h"
#include "stm32g4xx_hal.h"
FLASH_TypeDef fake_flash;
extern const launchcore_storage_driver_t launchcore_board_storage_driver;
int main(void)
{
    void *flash = mmap((void *)LAUNCHCORE_INTERNAL_FLASH_BASE, LAUNCHCORE_INTERNAL_FLASH_SIZE,
        PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    assert(flash == (void *)LAUNCHCORE_INTERNAL_FLASH_BASE);
    memset(flash, 0x5A, LAUNCHCORE_INTERNAL_FLASH_SIZE);
    const launchcore_storage_driver_t *d = &launchcore_board_storage_driver;
    const uint8_t record[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

    /* First provisioning operation after a DFU ':leave' handoff. */
    fake_flash.SR = FLASH_FLAG_OPTVERR | FLASH_FLAG_PGSERR;
    assert(d->erase(BOARD_PERSIST_BASE, FLASH_PAGE_SIZE) == LAUNCHCORE_STORAGE_OK);
    fake_flash.SR = FLASH_FLAG_OPTVERR;
    assert(d->write(BOARD_PERSIST_BASE, record, sizeof record) == LAUNCHCORE_STORAGE_OK);
    assert(!memcmp((const void *)BOARD_PERSIST_BASE, record, sizeof record));

    /* A real failure of the operation itself is still reported. */
    assert(d->write(BOARD_PERSIST_BASE, record, sizeof record) == LAUNCHCORE_STORAGE_ERR_WRITE);
    return 0;
}
'''


class FlashStaleStatusTests(unittest.TestCase):
    def test_stale_flash_status_does_not_fail_provisioning(self):
        launchcore = sorted((ROOT / "build").glob("*/_deps/sedslaunchcore-src/bootloader/include"))
        if not launchcore:
            self.skipTest("LaunchCore headers are fetched by a firmware build")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "stm32g4xx_hal.h").write_text(HAL)
            (path / "main.c").write_text(MAIN)
            exe = path / "flash"
            subprocess.run(
                ["cc", "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-no-pie",
                 "-I", str(path), "-I", str(ROOT / "Bootloader"), "-I", str(launchcore[0]),
                 str(path / "main.c"), str(ROOT / "Bootloader/storage_internal_flash.c"),
                 "-o", str(exe)],
                check=True)
            subprocess.run([str(exe)], check=True, timeout=10)


if __name__ == "__main__":
    unittest.main()
