#pragma once
#include <stdint.h>
uint32_t HAL_GetTick(void);void NVIC_SystemReset(void);
uint32_t __get_PRIMASK(void);void __disable_irq(void);void __set_PRIMASK(uint32_t);
