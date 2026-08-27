#include "board_config.h"
#include "launchcore/platform.h"
#include "launchcore/storage.h"
#include "stm32g4xx.h"

extern const launchcore_storage_driver_t launchcore_board_storage_driver;

void platform_early_init(void)
{
    launchcore_storage_set_driver(&launchcore_board_storage_driver);
}

void platform_clock_init(void) {}
void platform_external_ram_init(void) {}
void platform_external_flash_init(void) {}
void platform_deinit_before_jump(void) {}
bool platform_recovery_requested(void) { return false; }

void platform_system_reset(void)
{
    NVIC_SystemReset();
    for (;;) {}
}

uint32_t platform_get_reset_reason(void) { return RCC->CSR; }
void platform_feed_watchdog(void) {}

bool platform_validate_app_vector(uint32_t vector_table_addr, uint32_t stack_pointer,
                                  uint32_t reset_handler)
{
    const uint32_t app_end = BOARD_SLOT_A_BASE + BOARD_SLOT_A_SIZE;
    const bool vector_ok = vector_table_addr == BOARD_VECTOR_TABLE;
    const bool stack_ok = stack_pointer >= LAUNCHCORE_INTERNAL_SRAM_BASE &&
                          stack_pointer <= LAUNCHCORE_INTERNAL_SRAM_BASE +
                                               LAUNCHCORE_INTERNAL_SRAM_SIZE;
    const bool reset_ok = reset_handler >= BOARD_VECTOR_TABLE && reset_handler < app_end &&
                          (reset_handler & 1u) != 0u;
    return vector_ok && stack_ok && reset_ok;
}

/* The flash HAL only needs a monotonic timeout source in the bootloader. */
uint32_t HAL_GetTick(void)
{
    static uint32_t tick;
    return tick++;
}
