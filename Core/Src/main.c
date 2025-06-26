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
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "lptim.h"
#include "quadspi.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensor2_i2c.h"

#include "smtc_hal_dbg_trace.h"

#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"

#include "llcc68.h"
#include "llcc68_context.h"
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

/* USER CODE BEGIN PV */

extern uint8_t uart_busy;
extern uint32_t buf_head, buf_tail;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

void hal_mcu_set_sleep_for_ms(int32_t milliseconds);
uint32_t rtc_time_to_ms(const RTC_TimeTypeDef *t, uint32_t prediv_s);
int32_t rtc_diff_ms(const RTC_TimeTypeDef *t1,
                    const RTC_TimeTypeDef *t2,
                    uint32_t prediv_s);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void convert_and_print_serial(uint16_t* serial_raw) {
    uint64_t serial_as_int = 0;
    sensirion_common_to_integer((uint8_t*)serial_raw, (uint8_t*)&serial_as_int,
                                LONG_INTEGER, 6);
    SMTC_HAL_TRACE_PRINTF("0x%x", serial_as_int);
}

llcc68_context_t radio_context = {
    .spi = &hspi1,
    .nss_port = LORA_NSS_GPIO_Port,
    .nss_pin = LORA_NSS_Pin,
    .reset_port = LORA_NRST_GPIO_Port,
    .reset_pin = LORA_NRST_Pin,
    .busy_port = LORA_BUSY_GPIO_Port,
    .busy_pin = LORA_BUSY_Pin
};

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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_LPTIM1_Init();
  MX_LPTIM2_Init();
  MX_QUADSPI_Init();
  MX_RNG_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  hal_trace_print_init(&huart1);

  HAL_StatusTypeDef status;
  uint8_t I2Ccount=0;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Включение DWT
  DWT->CYCCNT = 0; // Сброс счётчика
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Включение счётчика циклов

  SMTC_HAL_TRACE_INFO("Hullo there!\n");
  SMTC_HAL_TRACE_INFO("Initializing I2C\n");

  I2C_HandleTypeDef *i2c_handles[3] = {&hi2c1, &hi2c2, &hi2c3};

  status = I2C_Sensor_Init(i2c_handles, 3);

  if (status != HAL_OK) {
	  SMTC_HAL_TRACE_ERROR("FUCK! Failed to init i2c lib \n");
  }

  status = I2C_Sensor_Discover_Devices(&I2Ccount);
  if (status != HAL_OK) {
  	  SMTC_HAL_TRACE_ERROR("FUCK! Failed to count i2c devices \n");
  }

  HAL_Delay(100);

  HAL_GPIO_WritePin(EN_5V_GPIO_Port, EN_5V_Pin, 0);



  RTC_TimeTypeDef last_time = {0};  // Время последнего измерения
  RTC_TimeTypeDef now_time = {0};
  RTC_DateTypeDef placeholder_date = {0};

  HAL_RTC_GetTime(&hrtc, &now_time, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &placeholder_date, RTC_FORMAT_BIN);
  last_time = now_time;

  uint32_t delta, delta2;
  uint32_t integralDelta = 0;

  uint8_t outData[51];


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	for (int i=0; i<10; i++){
		HAL_RTC_GetTime(&hrtc, &now_time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &placeholder_date, RTC_FORMAT_BIN);

		delta = rtc_diff_ms(&last_time, &now_time, 1023);
		integralDelta += delta;

		last_time = now_time;

		//SMTC_HAL_TRACE_INFO("Delta: %d\n", delta);

		I2C_Sensor_Run(delta);

		uint8_t arrsize = 0;

		if (integralDelta > 60*1000) {
			I2C_Sensor_Fetch_Data(outData, &arrsize);
			SMTC_HAL_TRACE_ARRAY("Data out:",outData,arrsize);
			integralDelta = 0;
		}

		HAL_Delay(10);
	}


	__disable_irq();
	hal_mcu_set_sleep_for_ms(10000);
	__enable_irq();


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() != 0)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_PWR_EnableBkUpAccess();
  LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_MEDIUMLOW);
  LL_RCC_LSE_Enable();

   /* Wait till LSE is ready */
  while(LL_RCC_LSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_1, 20, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(80000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  LL_RCC_PLLSAI1_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_1, 16, LL_RCC_PLLSAI1Q_DIV_4);
  LL_RCC_PLLSAI1_ConfigDomain_ADC(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_1, 16, LL_RCC_PLLSAI1R_DIV_2);
  LL_RCC_PLLSAI1_EnableDomain_48M();
  LL_RCC_PLLSAI1_EnableDomain_ADC();
  LL_RCC_PLLSAI1_Enable();

   /* Wait till PLLSAI1 is ready */
  while(LL_RCC_PLLSAI1_IsReady() != 1)
  {

  }
}

/* USER CODE BEGIN 4 */

void hal_mcu_set_sleep_for_ms(int32_t milliseconds)
{
    HAL_StatusTypeDef rtcStatus = HAL_OK;

    if (milliseconds <= 0) {
        return;
    }

    // 1. Дождаться завершения всех DMA-передач по UART
	// Таймаут 5 мс (достаточно для 20 байт при 460800 бит/с = ~0.43 мс)
	// SystemCoreClock - частота процессора в Гц, делим на 1000 для мс
	const uint32_t cycles_per_ms = SystemCoreClock / 1000;
	const uint32_t timeout_cycles = 5 * cycles_per_ms; // 5 мс
	volatile uint32_t count = timeout_cycles;

	__enable_irq();

	while (uart_busy && count > 0) {
		__NOP(); // Инструкция NOP
		count--;
	}

    if (uart_busy) {
        // Принудительно завершаем DMA, если таймаут истёк
        HAL_DMA_Abort(huart1.hdmatx);
        uart_busy = 0;
    }

    __disable_irq();

    buf_head = 0;
	buf_tail = 0;
	uart_busy = 0;

    // 2. Отключение DMA для UART
    if (huart1.hdmatx != NULL) {
    	HAL_UART_DMAStop(&huart1);
    	HAL_DMA_Abort(huart1.hdmatx);
        HAL_DMA_DeInit(huart1.hdmatx); // Деинициализируем DMA-канал
    }

    // 3. Настройка RTC Wake-Up Timer для пробуждения
    uint32_t delay_ms_2_tick = milliseconds * 2 + ((6 * milliseconds) >> 7);

    rtcStatus = HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    if (rtcStatus != HAL_OK) {
        return;
    }

    rtcStatus = HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, delay_ms_2_tick, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
    if (rtcStatus != HAL_OK) {
        return;
    }

    // 4. Отключение ненужной периферии перед входом в STOP2
    HAL_SuspendTick();
    HAL_SPI_DeInit(&hspi1);
    HAL_ADC_DeInit(&hadc1);
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_DeInit(&hi2c2);
    HAL_I2C_DeInit(&hi2c3);
    HAL_QSPI_DeInit(&hqspi);
    HAL_RNG_DeInit(&hrng);
    HAL_UART_DeInit(&huart1);
    HAL_UART_DeInit(&huart2);
    HAL_UART_DeInit(&huart3);
    HAL_CRC_DeInit(&hcrc);

    // 5. Обеспечение завершения всех операций
    __DSB();
    __ISB();

    // 6. Вход в режим STOP2
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

    // 7. Восстановление после выхода из STOP2
    SystemClock_Config();
    PeriphCommonClock_Config();

    // 8. Инициализация периферии
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_QUADSPI_Init();
    MX_RNG_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init(); // Уже включает инициализацию DMA
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_CRC_Init();

    // 9. Восстановление кольцевого буфера
    uart_busy = 0; // Сбрасываем флаг занятости
    if (buf_head != buf_tail) {
        // Если в буфере остались данные, запускаем передачу
        start_next_transmission();
    }

    HAL_ResumeTick();

    // 10. Отключение RTC Wake-Up Timer
    rtcStatus = HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
}

uint32_t rtc_time_to_ms(const RTC_TimeTypeDef *t, uint32_t prediv_s)
{
    /* Секундная часть */
    uint32_t ms = ((uint32_t)t->Hours   * 3600UL +
                   (uint32_t)t->Minutes *   60UL +
                   (uint32_t)t->Seconds) * 1000UL;

    /* Дробная часть: SubSeconds убывает! */
    uint32_t sub_ms = ((prediv_s - t->SubSeconds) * 1000UL) / (prediv_s + 1UL);

    return ms + sub_ms;
}

/* Возвращает Δt в миллисекундах (signed 32-bit); корректно работает с переходом через полночь */
int32_t rtc_diff_ms(const RTC_TimeTypeDef *t1,
                    const RTC_TimeTypeDef *t2,
                    uint32_t prediv_s)
{
    int32_t ms1 = (int32_t)rtc_time_to_ms(t1, prediv_s);
    int32_t ms2 = (int32_t)rtc_time_to_ms(t2, prediv_s);

    int32_t diff = ms2 - ms1;

    /* Если перешли через 24 ч, поправляем на полный день (86 400 000 мс) */
    if (diff < -43200000)       diff += 86400000;   // назад через полночь
    else if (diff > 43200000)   diff -= 86400000;   // вперёд через полночь

    return diff;
}

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
