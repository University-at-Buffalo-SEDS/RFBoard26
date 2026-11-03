#pragma once
#include <stdbool.h>
#include <stdint.h>
uint64_t board_commissioning_query_generation(void);
int32_t board_recovery_commission_prepare(uint32_t);
int32_t board_recovery_commission_apply(uint32_t);
bool board_recovery_commission_finish(uint32_t,uint32_t);
