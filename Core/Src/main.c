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
#include "rtc.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IIC3.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Clock Configuration Selection */
#define CONFIG_HSE 0
#if (CONFIG_HSE == 1)
#define CLOCK_CONFIG_HSE_LSE_ONLY 1 // 外部高速 + 外部低速: HSE + LSE
#define CLOCK_CONFIG_USE_HSE_PLL 1  // HSE 经 PLL 作为 SYSCLK
#else
#define CLOCK_CONFIG_HSI_LSE_ONLY 1 // 内部高速 + 外部低速: HSI + LSE
#define CLOCK_CONFIG_USE_HSI_PLL 1  // HSI 经 PLL 作为 SYSCLK
#endif
#if defined(STM32WB15xx) || defined(STM32WB10xx)
// 可支持 LSI
#define CLOCK_CONFIG_HSE_LSI_ONLY 0 // 外部高速 + 内部低速: HSE + LSI
#define CLOCK_CONFIG_HSI_LSI_ONLY 0 // 内部高速 + 内部低速: HSI + LSI
#define RF_WAKEUP_CLK_LSI1 1        // 选择 LSI1 作为 RF 唤醒时钟
#else
#define CLOCK_CONFIG_HSE_LSI_ONLY 0 // 外部高速 + 内部低速: HSE + LSI
#define CLOCK_CONFIG_HSI_LSI_ONLY 0 // 内部高速 + 内部低速: HSI + LSI
#endif
#define CLOCK_CONFIG_HSI_INCLUDED 0 // 启用HSI48 (USB/RNG) 实验模式

/* ============ 低速时钟 (LSE / LSI) 选择宏 ============
 * 规则: 仅允许 LSE 或 LSI 一种低速源被选用
 */
#if ((CLOCK_CONFIG_HSE_LSE_ONLY) + (CLOCK_CONFIG_HSI_LSE_ONLY)) && ((CLOCK_CONFIG_HSE_LSI_ONLY) + (CLOCK_CONFIG_HSI_LSI_ONLY))
#error "不可同时选择 LSE 与 LSI 组合"
#endif

/* ================= 编译期配置一致性检查 ================= */
#if ((CLOCK_CONFIG_HSE_LSE_ONLY) + (CLOCK_CONFIG_HSI_LSE_ONLY) + (CLOCK_CONFIG_HSI_INCLUDED) + (CLOCK_CONFIG_HSE_LSI_ONLY) + (CLOCK_CONFIG_HSI_LSI_ONLY)) != 1
#error "必须且只允许一个主配置宏为1: HSE_LSE_ONLY / HSI_LSE_ONLY / HSI_INCLUDED / HSE_LSI_ONLY / HSI_LSI_ONLY"
#endif

#if (CLOCK_CONFIG_HSE_LSE_ONLY == 1 || CLOCK_CONFIG_HSE_LSI_ONLY == 1) && (CLOCK_CONFIG_USE_HSI_PLL == 1)
#error "选择 HSE* 模式时不应启用 CLOCK_CONFIG_USE_HSI_PLL"
#endif
#if (CLOCK_CONFIG_HSI_LSE_ONLY == 1 || CLOCK_CONFIG_HSI_LSI_ONLY == 1 || CLOCK_CONFIG_HSI_INCLUDED == 1) && (CLOCK_CONFIG_USE_HSE_PLL == 1)
#error "选择 HSI* 模式时不应启用 CLOCK_CONFIG_USE_HSE_PLL"
#endif
#if (CLOCK_CONFIG_HSI_INCLUDED == 1) && (CLOCK_CONFIG_USE_HSE_PLL == 1) && (CLOCK_CONFIG_USE_HSI_PLL == 1)
#error "HSI_INCLUDED 模式下不应同时打开 HSE 与 HSI 的 PLL 宏"
#endif

/* ============ 统一低速时钟宏 ============ */
#if (CLOCK_CONFIG_HSE_LSE_ONLY || CLOCK_CONFIG_HSI_LSE_ONLY || CLOCK_CONFIG_HSI_INCLUDED)
#define LOW_SPEED_IS_LSE 1
#else
#define LOW_SPEED_IS_LSE 0
#endif

#if LOW_SPEED_IS_LSE
#define LSE_STATE_CONFIG RCC_LSE_ON
#define LSI_STATE_CONFIG RCC_LSI_OFF
#define RF_WAKEUP_CLK RCC_RFWKPCLKSOURCE_LSE
#else
#define LSE_STATE_CONFIG RCC_LSE_OFF
#define LSI_STATE_CONFIG RCC_LSI_ON
#ifdef RF_WAKEUP_CLK_LSI1
#define RF_WAKEUP_CLK RCC_OSCILLATORTYPE_LSI1
#else
#define RF_WAKEUP_CLK RCC_OSCILLATORTYPE_LSI2
#endif
#endif

/* ============ SMPS 时钟源选择 ============
 * 原代码固定使用 HSI 作为 SMPS 时钟，但在 HSE_LSE_ONLY 模式下已关闭 HSI，
 * 会导致 SMPS 时钟不可用 -> 无线子系统可能异常，表现为 BLE 不广播/不连接。
 * 规则:
 *  - 如果 HSE 打开且 HSI 关闭: 使用 HSE 作为 SMPS 时钟
 *  - 其余情况: 使用 HSI (因为 HSI 仍保持开启)
 */
#if (defined(HSE_STATE_CONFIG) && (HSE_STATE_CONFIG == RCC_HSE_ON) && defined(HSI_STATE_CONFIG) && (HSI_STATE_CONFIG == RCC_HSI_OFF))
#define SMPS_CLK_SRC_CONFIG RCC_SMPSCLKSOURCE_HSE
#else
#define SMPS_CLK_SRC_CONFIG RCC_SMPSCLKSOURCE_HSI
#endif

/* 根据配置选择不同的时钟设置 */
#if (CLOCK_CONFIG_HSE_LSE_ONLY == 1)
// 模式1: HSE(32MHz)+LSE(32.768kHz)
#define OSC_TYPE_CONFIG (RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE)
#define HSE_STATE_CONFIG RCC_HSE_ON
#define HSI_STATE_CONFIG RCC_HSI_OFF
#define HSI48_CONFIG_ENABLE 0
#define SYSCLK_SOURCE_CONFIG RCC_SYSCLKSOURCE_HSE
#define FLASH_LATENCY_CONFIG FLASH_LATENCY_2
#define CONFIG_COMMENT "/* 配置: HSE+LSE */"
#elif (CLOCK_CONFIG_HSI_LSE_ONLY == 1)
// 模式2: HSI(16MHz/PLL后32MHz)+LSE
#define OSC_TYPE_CONFIG (RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE)
#define HSE_STATE_CONFIG RCC_HSE_OFF
#define HSI_STATE_CONFIG RCC_HSI_ON
#define HSI48_CONFIG_ENABLE 0
#define SYSCLK_SOURCE_CONFIG RCC_SYSCLKSOURCE_HSI
#define FLASH_LATENCY_CONFIG FLASH_LATENCY_2
#define CONFIG_COMMENT "/* 配置: HSI+LSE */"
#elif (CLOCK_CONFIG_HSE_LSI_ONLY == 1)
// 模式3: HSE + LSI (去掉外部32k, 牺牲RTC精度)
#define OSC_TYPE_CONFIG (RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI1)
#define HSE_STATE_CONFIG RCC_HSE_ON
#define HSI_STATE_CONFIG RCC_HSI_OFF
#define HSI48_CONFIG_ENABLE 0
#define SYSCLK_SOURCE_CONFIG RCC_SYSCLKSOURCE_HSE
#define FLASH_LATENCY_CONFIG FLASH_LATENCY_2
#define CONFIG_COMMENT "/* 配置: HSE+LSI (低成本, RTC精度下降) */"
#elif (CLOCK_CONFIG_HSI_LSI_ONLY == 1)
// 模式4: HSI + LSI (最低BOM, RTC误差最大)
#define OSC_TYPE_CONFIG (RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI1)
#define HSE_STATE_CONFIG RCC_HSE_OFF
#define HSI_STATE_CONFIG RCC_HSI_ON
#define HSI48_CONFIG_ENABLE 0
#define SYSCLK_SOURCE_CONFIG RCC_SYSCLKSOURCE_HSI
#define FLASH_LATENCY_CONFIG FLASH_LATENCY_2
#define CONFIG_COMMENT "/* 配置: HSI+LSI (最小BOM) */"
#elif CLOCK_CONFIG_HSI_INCLUDED
// 模式5: HSI + HSI48 (+ LSE 默认) 测试/需要RNG
#define OSC_TYPE_CONFIG (RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSI | (LOW_SPEED_IS_LSE ? RCC_OSCILLATORTYPE_LSE : RCC_OSCILLATORTYPE_LSI) | RCC_OSCILLATORTYPE_HSE)
#define HSE_STATE_CONFIG RCC_HSE_ON
#define HSI_STATE_CONFIG RCC_HSI_ON
#define HSI48_CONFIG_ENABLE 1
#define SYSCLK_SOURCE_CONFIG RCC_SYSCLKSOURCE_HSI
#define FLASH_LATENCY_CONFIG FLASH_LATENCY_2
#define CONFIG_COMMENT "/* 配置: HSI+HSI48 (+低速) */"
#else
#error "请选择一个有效的时钟配置!"
#endif

#if CLOCK_CONFIG_USE_HSE_PLL && (CLOCK_CONFIG_HSE_LSE_ONLY || CLOCK_CONFIG_HSE_LSI_ONLY)
// PLL: HSE -> PLL -> 32MHz
#define PLL_STATE_CONFIG RCC_PLL_ON
#define PLL_SOURCE_CONFIG RCC_PLLSOURCE_HSE
#define PLL_M_CONFIG RCC_PLLM_DIV2
#define PLL_N_CONFIG 8
#define PLL_P_CONFIG RCC_PLLP_DIV4
#define PLL_R_CONFIG RCC_PLLR_DIV4
#define PLL_Q_CONFIG RCC_PLLQ_DIV4
#undef SYSCLK_SOURCE_CONFIG
#define SYSCLK_SOURCE_CONFIG RCC_SYSCLKSOURCE_PLLCLK
#undef FLASH_LATENCY_CONFIG
#define FLASH_LATENCY_CONFIG FLASH_LATENCY_1
#elif CLOCK_CONFIG_USE_HSI_PLL && (CLOCK_CONFIG_HSI_LSE_ONLY || CLOCK_CONFIG_HSI_LSI_ONLY || CLOCK_CONFIG_HSI_INCLUDED)
// PLL: HSI -> PLL -> 32MHz
#define PLL_STATE_CONFIG RCC_PLL_ON
#define PLL_SOURCE_CONFIG RCC_PLLSOURCE_HSI
#define PLL_M_CONFIG RCC_PLLM_DIV1
#define PLL_N_CONFIG 8
#define PLL_P_CONFIG RCC_PLLP_DIV4
#define PLL_R_CONFIG RCC_PLLR_DIV4
#define PLL_Q_CONFIG RCC_PLLQ_DIV4
#undef SYSCLK_SOURCE_CONFIG
#define SYSCLK_SOURCE_CONFIG RCC_SYSCLKSOURCE_PLLCLK
#undef FLASH_LATENCY_CONFIG
#define FLASH_LATENCY_CONFIG FLASH_LATENCY_1
#else
#define PLL_STATE_CONFIG RCC_PLL_NONE
#endif

/* 模式快速说明:
 * 1 HSE+LSE: 精准主频+精准RTC, 功耗最低 (需2晶振)
 * 2 HSI+LSE: 减少高速晶振, RTC精度保留
 * 3 HSE+LSI: 保留高速精度, 放弃RTC精度
 * 4 HSI+LSI: 最低成本, RTC漂移大
 * 5 HSI+HSI48(+LSE/LSI): 需要RNG/USB
 * 选择 PLL 宏可把主频固定到 32MHz
 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void Enter_Low_Power_Stop0_Mode(void);
static void EnterSleepForSeconds(uint32_t seconds); // 新增：普通 Sleep 功耗测试
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t cnt = 0;
  while (1)
  {
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET); // 片选
    uint8_t cmd[20] = {0x9F, 0x00, 0x00, 0x00, 0x9F, 0x00, 0x00, 0x00, 0x9F, 0x00, 0x00, 0x00, 0x9F, 0x00, 0x00, 0x00, 0x9F, 0x00, 0x00, 0x00};
    HAL_SPI_Transmit(&hspi1, cmd, 20, 1000);
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET); // 取消片选
    // HAL_SPI_Transmit(&hspi1, &cnt, 1, 1000);
    //  改为进入 STOP2 模式：1 秒深度睡眠
     HAL_Delay(4000);
    // EnterSleepForSeconds(4);
    //  Enter_Low_Power_Stop0_Mode();
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

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = OSC_TYPE_CONFIG;
  RCC_OscInitStruct.HSEState = HSE_STATE_CONFIG;
  RCC_OscInitStruct.LSEState = LSE_STATE_CONFIG;
  RCC_OscInitStruct.HSIState = HSI_STATE_CONFIG;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = PLL_STATE_CONFIG;
  RCC_OscInitStruct.PLL.PLLSource = PLL_SOURCE_CONFIG;
  RCC_OscInitStruct.PLL.PLLM = PLL_M_CONFIG;
  RCC_OscInitStruct.PLL.PLLN = PLL_N_CONFIG;
  RCC_OscInitStruct.PLL.PLLP = PLL_P_CONFIG;
  RCC_OscInitStruct.PLL.PLLR = PLL_R_CONFIG;
  RCC_OscInitStruct.PLL.PLLQ = PLL_Q_CONFIG;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4 | RCC_CLOCKTYPE_HCLK2 | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = SYSCLK_SOURCE_CONFIG;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
   */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = SMPS_CLK_SRC_CONFIG;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}


/* USER CODE BEGIN 4 */
extern RTC_HandleTypeDef hrtc;
static volatile uint8_t g_rtc_wakeup_flag; // 外部声明唤醒标志
/* 普通 Sleep: 不置 SLEEPDEEP，只关闭 SysTick，并用 RTC WakeUp 作为唤醒源。
 * 目的：测量 Sleep (CPU clock gate) 而非 Stop 模式下的功耗基线。
 */
static void EnterSleepForSeconds(uint32_t seconds)
{
  if (seconds == 0)
    return;
  HAL_SuspendTick();
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
#if defined(RTC_WAKEUPCLOCK_CK_SPRE_16BITS)
  const uint32_t wake_clk = RTC_WAKEUPCLOCK_RTCCLK_DIV16; // 1Hz
                                                          /*
                                                              1. 技术原理差异
                                                          RTC_WAKEUPCLOCK_RTCCLK_DIV16  更加精确，没有锯齿漂移
                                                          时钟源: RTCCLK / 16 = 32.768kHz / 16 = 2048Hz
                                                          分辨率: 1/2048秒 = 0.48828ms
                                                          计数范围: 0-65535 (16位)
                                                          最大延时: 65535/2048 ≈ 32秒
                                                          RTC_WAKEUPCLOCK_CK_SPRE_16BITS 有锯齿漂移
                                                          时钟源: 使用RTC子系统内部的1Hz时钟（来自预分频器链）
                                                          分辨率: 1秒
                                                          计数范围: 0-65535
                                                          最大延时: 65535秒 ≈ 18.2小时
                                                          */
#elif defined(RTC_WAKEUPCLOCK_CK_SPRE)
  const uint32_t wake_clk = RTC_WAKEUPCLOCK_CK_SPRE; // 1Hz (旧宏)
#else
#error "No 1Hz wakeup clock macro defined for this HAL."
#endif
  if (seconds > 0xFFFF)
    seconds = 0xFFFF;
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, seconds * 2048 - 1, wake_clk) != HAL_OK)
  {
    HAL_ResumeTick();
    return; // WakeUp 设置失败
  }

  g_rtc_wakeup_flag = 0; // 清除唤醒标志

  // 使用 HAL 库函数进入普通 Sleep 模式 (不进入 STOP)
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  // 唤醒后清理
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  HAL_ResumeTick();
}

/**
 * @brief 启动RTC唤醒定时器 - 进入睡眠前调用
 * @retval None
 */
void RTC_Start_WakeUpTimer(void)
{
  /* 确保RTC唤醒定时器是停止状态 */
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  /* 清除RTC唤醒标志位 */
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

  /* 启动RTC唤醒定时器：4秒后唤醒 */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 8191, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }

  /* 启用NVIC中断：优先级0，确保能正常响应唤醒中断 */
  HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
}

/**
 * @brief 停止RTC唤醒定时器 - 从睡眠唤醒后调用
 * @retval None
 */
void RTC_Stop_WakeUpTimer(void)
{
  /* 停止RTC唤醒定时器 */
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  /* 禁用RTC唤醒中断 */
  HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);

  /* 清除可能残留的标志位 */
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
}

/**
 * @brief 进入Low Power STOP0模式
 * @retval None
 */
void Enter_Low_Power_Stop0_Mode(void)
{
  /* 挂起SysTick，避免睡眠期间被唤醒（测量更干净） */
  HAL_SuspendTick();

  /* 启动RTC唤醒定时器：4秒后唤醒 */
  RTC_Start_WakeUpTimer();

  /* 确保内存写入完成 */
  __DSB();
  __ISB();

  /* 进入STOP0模式：CPU停止运行，但SRAM保持供电，外设时钟停止 */
  // HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
  HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  /* 从STOP2模式唤醒后，需要重新配置系统时钟 */
  /* 因为STOP2 模式中HSI和PLL会被关闭，唤醒后默认使用MSI */
  // SystemClock_Config();

  /* 唤醒后停止唤醒定时器 */
  RTC_Stop_WakeUpTimer();

  /* 恢复SysTick */
  HAL_ResumeTick();
}

void RTC_WKUP_IRQHandler(void)
{
  HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  g_rtc_wakeup_flag = 1;
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
