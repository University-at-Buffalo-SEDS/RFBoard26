#pragma once
#include <stdint.h>
struct RccFake{uint32_t APB1ENR1;};struct PwrFake{uint32_t CR1;};
struct TampFake{uint32_t BKP28R,BKP29R,BKP30R,BKP31R;};
extern struct RccFake test_rcc;extern struct PwrFake test_pwr;extern struct TampFake test_tamp;
#define RCC (&test_rcc)
#define PWR (&test_pwr)
#define TAMP (&test_tamp)
#define RCC_APB1ENR1_PWREN 1U
#define RCC_APB1ENR1_RTCAPBEN 2U
#define PWR_CR1_DBP 4U
void test_dsb(void);
#define __DSB() test_dsb()
uint32_t HAL_GetTick(void);

#define __get_IPSR() 0U
