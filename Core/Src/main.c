/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"
#include <stdint.h>
#include <stdio.h>

extern UX_SLAVE_CLASS_CDC_ACM *cdc_acm;

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan2;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN2_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  MX_ThreadX_Init();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 10;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 13;
  hfdcan2.Init.NominalTimeSeg2 = 3;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GREEN_LED_Pin|BLUE_LEDS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GREEN_LED_Pin BLUE_LEDS_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin|BLUE_LEDS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static inline int cdc_in_isr(void) {
  // Same as before, works on ThreadX too
  return (__get_IPSR() != 0U);
}

/* Simple "kernel running" heuristic:
 * - Before tx_kernel_enter() returns and main thread starts,
 * tx_thread_identify() is NULL.
 * - In normal thread context, it returns the current TX_THREAD*.
 */
static inline int cdc_kernel_running(void) {
  return !cdc_in_isr() && (tx_thread_identify() != TX_NULL);
}

/* ------------ Mutex for multi-thread serialization ------------ */

static TX_MUTEX cdc_mutex;

void cdc_printf_init(void) {
  // Call this from your application init (e.g. the main ThreadX app thread)
  tx_mutex_create(&cdc_mutex, "cdc_tx_mutex", TX_NO_INHERIT);
}

static void cdc_lock(void) {
  // Don't try to lock from ISR or before the scheduler
  if (!cdc_kernel_running()) {
    return;
  }
  // We *could* use TX_NO_WAIT here, but serialization is cheap and fast
  tx_mutex_get(&cdc_mutex, TX_WAIT_FOREVER);
}

static void cdc_unlock(void) {
  if (!cdc_kernel_running()) {
    return;
  }
  tx_mutex_put(&cdc_mutex);
}

/* ------------ USB ready helper ------------ */

static inline UINT usb_cdc_ready(void) {
  // Simplest check: class instance is valid
  return (cdc_acm != UX_NULL);
}

/* ------------ Bounded, drop-on-trouble CDC write ------------ */

static void cdc_write_raw(const uint8_t *buf, uint16_t len) {
  if (!buf || !len || cdc_in_isr()) {
    return;
  }

  // If USB not configured/ready, just drop (prevents heap blowup upstream)
  if (!usb_cdc_ready()) {
    return;
  }

  cdc_lock();

  ULONG sent = 0;
  ULONG start_ticks = tx_time_get();
  const ULONG overall_budget_ms = 5; // overall budget (ms) per _write()
  const ULONG ticks_per_second = TX_TIMER_TICKS_PER_SECOND;
  const ULONG overall_budget_ticks =
      (overall_budget_ms * ticks_per_second + 999) / 1000; // ceil division

  while (sent < (ULONG)len) {
    // USB went away mid-write? abort
    if (!usb_cdc_ready()) {
      break;
    }

    // Respect overall time budget so we never stall the RTOS
    if ((tx_time_get() - start_ticks) >= overall_budget_ticks) {
      break; // drop the rest of this line
    }

    ULONG chunk = (ULONG)len - sent;
    if (chunk > 64u) {
      chunk = 64u; // USB FS packet size
    }

    ULONG actual = 0;
    UINT status = ux_device_class_cdc_acm_write(cdc_acm, (UCHAR *)&buf[sent],
                                                chunk, &actual);

    if (status != UX_SUCCESS || actual == 0u) {
      // Host not ready or error → don't fight it, drop remainder
      break;
    }

    sent += actual;
    // ux_device_class_cdc_acm_write() is synchronous; USBX handles
    // packetization.
  }

  cdc_unlock();
}

// ---------- printf redirection ----------

#ifdef __GNUC__
int _write(int file, char *ptr, int len) {
  (void)file;
  if (len <= 0) {
    return 0;
  }

  // If USB not configured, pretend we consumed everything (best-effort debug)
  if (!usb_cdc_ready()) {
    return len;
  }

  uint8_t buf[128];
  int i = 0;

  // Track if the last character we *sent* was '\r'
  static uint8_t last_was_cr = 0;

  while (i < len) {
    uint16_t w = 0;

    while (i < len && w < sizeof(buf)) {
      uint8_t c = (uint8_t)ptr[i++];

      if (c == '\n') {
        // We want: "\r\n" if previous char was NOT '\r'
        if (!last_was_cr) {
          // Make sure we have space for both '\r' and '\n'
          if (w > (uint16_t)(sizeof(buf) - 2)) {
            // Not enough room; back up one and flush this buffer
            i--; // re-process this '\n' in the next iteration
            break;
          }
          buf[w++] = '\r';
        }

        // Now write '\n'
        if (w >= sizeof(buf)) {
          // No more room; back up one and flush
          i--;
          break;
        }
        buf[w++] = '\n';
        last_was_cr = 0; // last emitted was '\n'
      } else {
        // Normal char (could be '\r' itself)
        if (w >= sizeof(buf)) {
          // No room, back up one char and flush buffer
          i--;
          break;
        }
        buf[w++] = c;
        last_was_cr = (c == '\r');
      }
    }

    if (w > 0) {
      cdc_write_raw(buf, w);
    }
  }

  return len;
}
#else
int fputc(int ch, FILE *f) {
  (void)f;

  static uint8_t last_was_cr = 0;
  uint8_t buf[2];
  uint16_t w = 0;

  if (!usb_cdc_ready()) {
    return ch;
  }

  if (ch == '\n') {
    if (!last_was_cr) {
      buf[w++] = '\r';
    }
    buf[w++] = '\n';
    last_was_cr = 0;
  } else {
    buf[w++] = (uint8_t)ch;
    last_was_cr = (ch == '\r');
  }

  if (w > 0) {
    cdc_write_raw(buf, w);
  }
  return ch;
}
#endif
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
