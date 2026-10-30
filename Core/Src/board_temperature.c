#include "board_temperature.h"
#include "board_temperature_math.h"
#include "main.h"
#include <stdbool.h>
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_bus.h"
#define ADC_COMMON ADC12_COMMON
#define TEMP_CHANNEL LL_ADC_CHANNEL_TEMPSENSOR_ADC1
enum {
  TEMP_INIT_MASK = 3U,
  TEMP_INIT_READY = 1U,
  TEMP_INIT_FAILED = 2U,
  TEMP_VALID = 4U,
  TEMP_ATTEMPTED = 8U,
  TEMP_BUSY = 16U,
};
typedef struct {
  uint32_t last_attempt;
  int16_t sample_centi;
  uint8_t flags;
} board_temperature_state_t;
_Static_assert(sizeof(board_temperature_state_t) == 8U, "bounded temperature state");
static board_temperature_state_t temperature_state;

static bool due(uint32_t now)
{
  const uint32_t irq = __get_PRIMASK(); __disable_irq();
  if ((temperature_state.flags & TEMP_BUSY) ||
      ((temperature_state.flags & TEMP_ATTEMPTED) &&
       (uint32_t)(now - temperature_state.last_attempt) < 5000U)) {
    __set_PRIMASK(irq); return false;
  }
  /* Keep the prior successful timestamp until publish. A concurrent reader
   * still sees its original age, and a reentrant caller cannot sample twice. */
  temperature_state.flags |= TEMP_ATTEMPTED | TEMP_BUSY;
  __set_PRIMASK(irq); return true;
}
static void publish(uint32_t raw, uint32_t ref, uint32_t now)
{
  int16_t c;
  const bool ok = board_temperature_convert(raw, ref, *VREFINT_CAL_ADDR,
      *TEMPSENSOR_CAL1_ADDR, *TEMPSENSOR_CAL2_ADDR,
      TEMPSENSOR_CAL1_TEMP, TEMPSENSOR_CAL2_TEMP, &c);
  const uint32_t irq = __get_PRIMASK(); __disable_irq();
  /* A valid sample is necessarily from the last completed attempt. Failed
   * attempts clear validity, so one timestamp serves both cadence and age. */
  if (ok) temperature_state.sample_centi = c;
  temperature_state.last_attempt = now;
  temperature_state.flags = (uint8_t)((temperature_state.flags & ~(TEMP_VALID | TEMP_BUSY)) |
                                     (ok ? TEMP_VALID : 0U));
  __set_PRIMASK(irq);
}
uint32_t board_temperature_health_flags(void)
{
  const uint32_t now = HAL_GetTick();
  const uint32_t irq = __get_PRIMASK(); __disable_irq();
  const uint32_t flags = (temperature_state.flags & TEMP_VALID) ?
      board_temperature_pack(temperature_state.sample_centi,
                             now - temperature_state.last_attempt) : 0U;
  __set_PRIMASK(irq); return flags;
}

static void delay_us(uint32_t us)
{
  volatile uint32_t n = (SystemCoreClock / 1000000U + 1U) * us;
  while (n--) __NOP();
}
static bool wait_bits(volatile uint32_t *reg, uint32_t mask, bool set)
{
  uint32_t budget = SystemCoreClock / 10000U + 1U;
  while (((*reg & mask) != 0U) != set) if (--budget == 0U) return false;
  return true;
}
static bool initialize(void)
{
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
  /* VB shares ADC12 with pressure ADC2: never change its common clock. */
  LL_ADC_SetCommonClock(ADC_COMMON, LL_ADC_CLOCK_SYNC_PCLK_DIV4);
  LL_ADC_SetCommonPathInternalChAdd(ADC_COMMON, LL_ADC_PATH_INTERNAL_TEMPSENSOR | LL_ADC_PATH_INTERNAL_VREFINT);
  LL_ADC_DisableDeepPowerDown(ADC1);
  LL_ADC_EnableInternalRegulator(ADC1);
  delay_us(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US);
  LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
  LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
  LL_ADC_SetChannelSamplingTime(ADC1, TEMP_CHANNEL, LL_ADC_SAMPLINGTIME_640CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_640CYCLES_5);
  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
  if (!wait_bits(&ADC1->CR, ADC_CR_ADCAL, false)) return false;
  delay_us(2U);
  LL_ADC_ClearFlag_ADRDY(ADC1);
  LL_ADC_Enable(ADC1);
  if (!wait_bits(&ADC1->ISR, ADC_ISR_ADRDY, true)) return false;
  delay_us(LL_ADC_DELAY_TEMPSENSOR_STAB_US);
  return true;
}
static bool read_channel(uint32_t channel, uint32_t *out)
{
  if ((ADC1->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART)) != 0U) return false;
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, channel);
  LL_ADC_ClearFlag_EOC(ADC1);
  LL_ADC_REG_StartConversion(ADC1);
  if (!wait_bits(&ADC1->ISR, ADC_ISR_EOC, true)) return false;
  *out = LL_ADC_REG_ReadConversionData12(ADC1);
  return wait_bits(&ADC1->CR, ADC_CR_ADSTART, false);
}
void board_temperature_poll(void)
{
  uint32_t now = HAL_GetTick();
  if (!due(now)) return;
  uint32_t raw = 0U, ref = 0U;
  if ((temperature_state.flags & TEMP_INIT_MASK) == 0U) {
    const uint8_t initialized = initialize() ? TEMP_INIT_READY : TEMP_INIT_FAILED;
    const uint32_t irq = __get_PRIMASK(); __disable_irq();
    temperature_state.flags = (uint8_t)((temperature_state.flags & ~TEMP_INIT_MASK) | initialized);
    __set_PRIMASK(irq);
  }
  if ((temperature_state.flags & TEMP_INIT_MASK) == TEMP_INIT_READY &&
      read_channel(TEMP_CHANNEL, &raw) && read_channel(LL_ADC_CHANNEL_VREFINT, &ref)) publish(raw, ref, now);
  else publish(0U, 0U, now);
}
