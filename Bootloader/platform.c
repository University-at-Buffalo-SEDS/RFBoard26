#include "board_config.h"
#include "launchcore/platform.h"
#include "launchcore/storage.h"
#include "stm32g4xx.h"

extern const launchcore_storage_driver_t launchcore_board_storage_driver;

#define BOOT_STATUS_GREEN_PIN 11u
#define BOOT_STATUS_BLUE_PIN 15u

static void status_led_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;

    GPIOB->MODER = (GPIOB->MODER &
                    ~((3u << (BOOT_STATUS_GREEN_PIN * 2u)) |
                      (3u << (BOOT_STATUS_BLUE_PIN * 2u)))) |
                   (1u << (BOOT_STATUS_GREEN_PIN * 2u)) |
                   (1u << (BOOT_STATUS_BLUE_PIN * 2u));
    GPIOB->OTYPER &= ~((1u << BOOT_STATUS_GREEN_PIN) |
                       (1u << BOOT_STATUS_BLUE_PIN));
    GPIOB->PUPDR &= ~((3u << (BOOT_STATUS_GREEN_PIN * 2u)) |
                      (3u << (BOOT_STATUS_BLUE_PIN * 2u)));

    /* Blue means LaunchCore is running or has fallen into recovery. */
    GPIOB->BSRR = (1u << BOOT_STATUS_BLUE_PIN) |
                  (1u << (BOOT_STATUS_GREEN_PIN + 16u));
}

void platform_early_init(void)
{
    status_led_init();
    launchcore_storage_set_driver(&launchcore_board_storage_driver);
}

void platform_clock_init(void) {}
void platform_external_ram_init(void) {}
void platform_external_flash_init(void) {}
void platform_deinit_before_jump(void)
{
    /* Green marks a validated application jump; application GPIO init takes
     * ownership of both pins immediately afterward. */
    GPIOB->BSRR = (1u << BOOT_STATUS_GREEN_PIN) |
                  (1u << (BOOT_STATUS_BLUE_PIN + 16u));
}
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
    const bool vector_ok = vector_table_addr == BOARD_VECTOR_TABLE &&
                           (vector_table_addr & 0x1FFu) == 0u;
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
