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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <time.h>
#include "demos.h"
#include "ili9341.h"
#include "w25qxx.h"
#include "TP_operations.h"
#include "XPT2046_touch.h"
#include "TTF.H"
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
RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint8_t idx[13];
uint8_t idy[13];
extern lcdPropertiesTypeDef  lcdProperties;
extern uint16_t lcd_text_color ;
extern uint16_t lcd_background_color;
extern uint16_t Paint_Color;

uint8_t change_colors = 0;
uint8_t run_slideshow = 0;
uint8_t mode_night = 0;
uint8_t max_day; // maximum day in month
int32_t SECONDS_TO_CALIBRATE = 0;
uint32_t HOURS_CALIBRATE_PERIOD = 1;
uint8_t  UPDATE_FLAG = 0;
int32_t century = 21;

RTC_TimeTypeDef my_time;
RTC_TimeTypeDef old_time;
RTC_DateTypeDef my_date;

RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;
time_t timestamp;
struct tm currTime;


POINT   Pressed_Point ;

uint8_t LCD_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE;
uint8_t LCD_NOT_WORK_ORIENTATION ;
uint8_t LCD_PORTRAIT_WORK_ORIENTATION ;
uint8_t LCD_PORTRAIT_NOT_WORK_ORIENTATION ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_RNG_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
//================================================================
void lcd_setup_picture(uint8_t pic_nr);
uint32_t readPicFromFlash(uint8_t pic_nr);

extern void main_app(void);
extern void do_calibrate(void);
extern void start_Paint(void);
extern MATRIX  matrix ;
extern void TIM4_IRQHandler(void);

char* getDayofweek(uint8_t number);
int calc_dow(int y, int m,int d);
void set_RTC_DATE(void);

void colon_print(void);
void read_RTC(void);
void print_RTC(void);
void set_color_RTC(void);
void test_print_RTC(void);
void test_setup(void);
bool clear_ss(void);
bool set_hours(void);
void put_number(uint8_t data);
void get_pressed_point(void);
uint8_t test_action(void);
bool test_pressed_point(uint16_t xmin, uint16_t xmax, uint16_t ymin, uint16_t ymax);
bool set_minuts(void);
void check_RTC_DATE(void);
bool set_year(void);
void put_DATE_number(void);
bool set_month(void);
bool set_day(void);
bool setting_globals(void);
void wait_for_releasseTS(void);
void paint_test(void);
void TTF_test(void);
void set_LIGHT(void);
void set_PWM(void);
void COLOR_FONTS(void);
void night_mode(void);
void check_night_mode(void);
void CHECK_CALIBRATE_TIME_VALUES(void);
void TIM_CALIBRATE_SETUP(void);
uint8_t calibr_action(void);
void next_page(void);
void test_page(void);
void CAL_TS(void) ;
void SLIDE_SHOW(void);
void DEMOS_RUN(void);
void V_BALL(void);
void check_work_orientation(void);
void ROTATION_TEST(void);
void slideshow_disable(void);
void check_slideshow(void);
void night_light(void);
void set_night_PWM(void);
void set_EPOCH(void);
void compare_EPOCH();
void read_timestamp(void);
void SET_CENTURY(void);
void READ_CENTURY(void);


bool go_to_back(void);


//TODO declared functions
//=================================================================
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
  MX_FSMC_Init();
  MX_RNG_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //===========================================
  check_work_orientation();
  check_slideshow();
  HAL_TIM_Base_Stop_IT(&htim4); // stop INTERRUPT from TIMER4 for colon blink
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // enable PWM back light
  change_colors = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
  mode_night = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
  READ_CENTURY();
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 65535);
  CHECK_CALIBRATE_TIME_VALUES(); // check calibration time values and setup if needed
  //===========================================
  LCD_ILI9341_init();
  lcdSetOrientation(LCD_WORK_ORIENTATION);
  //====================================================

  lcd_setup_picture(7);
  readPicFromFlash(7);

  for (int t = 65535; t > 0; t -= 32)
  {
	  HAL_Delay(1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, t);
  }

  LCD_ClrScr(0);
  HAL_Delay(500);


  set_PWM();
  print_RTC();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  test_print_RTC();
	  test_setup();

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8400-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8400-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FLASH_CS_Pin|T_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PE2 PE4 PE5 PE6
                           PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY1_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC0 PC1 PC2
                           PC3 PC4 PC6 PC7
                           PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA8 PA9 PA10 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : T_PEN_Pin */
  GPIO_InitStruct.Pin = T_PEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(T_PEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB11 PB6
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : T_CS_Pin */
  GPIO_InitStruct.Pin = T_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(T_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD12 PD2 PD3
                           PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};
  FSMC_NORSRAM_TimingTypeDef ExtTiming = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_ENABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 60;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */
  ExtTiming.AddressSetupTime = 4;
  ExtTiming.AddressHoldTime = 15;
  ExtTiming.DataSetupTime = 5;
  ExtTiming.BusTurnAroundDuration = 0;
  ExtTiming.CLKDivision = 16;
  ExtTiming.DataLatency = 17;
  ExtTiming.AccessMode = FSMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */
//=====================================================================================================================
//=====================================================================================================================

//=====================================================================================================================

/**
 * @brief test key1 (PE3) or screen (PC5) pressed
 *
 * @return != 0 = PRESSED
 */
uint8_t test_stop(void) {
	uint8_t result = 0;
	if((KEY1_GPIO_Port->IDR & KEY1_Pin) == 0){ result = 1 ;} // key1 pressed?
	if((T_PEN_GPIO_Port->IDR & T_PEN_Pin) == 0){ result = 1 ;} // screen pressed ?
	if(result != 0){ HAL_Delay(20) ;}
	return result;
}
void lcd_setup_picture(uint8_t pic_nr)
{
	uint8_t dana ;
	lcdOrientationTypeDef dana2;

	lcdOrientationTypeDef setup_pic[] = {
			LCD_WORK_ORIENTATION,
			LCD_PORTRAIT_NOT_WORK_ORIENTATION,
			LCD_PORTRAIT_WORK_ORIENTATION,
			LCD_NOT_WORK_ORIENTATION,
			LCD_NOT_WORK_ORIENTATION,
			LCD_WORK_ORIENTATION,
			LCD_WORK_ORIENTATION,
			LCD_WORK_ORIENTATION,
#define GITH
#ifdef GITH
			LCD_PORTRAIT_NOT_WORK_ORIENTATION,
			LCD_PORTRAIT_WORK_ORIENTATION,
			LCD_NOT_WORK_ORIENTATION,
			LCD_NOT_WORK_ORIENTATION
#else

			LCD_NOT_WORK_ORIENTATION,
			LCD_NOT_WORK_ORIENTATION,
			LCD_NOT_WORK_ORIENTATION,
			LCD_NOT_WORK_ORIENTATION
#endif
	};

	dana = pic_nr;
	if (dana > 12) {dana =0;}
	dana2 =  setup_pic[dana];
	lcdSetOrientation(dana2);

	LCD_OpenWin(0, 0, lcdProperties.width, lcdProperties.height);


}
//=====================================================================================================================
uint32_t readPicFromFlash(uint8_t pic_nr)
{
	uint8_t page_data[256];
	uint32_t i, i2, ADDR_PIC;
	uint16_t* DATA_PAGE  = (uint16_t*) &page_data[0];
	uint32_t command = 0x03;
	uint8_t * tmpr1 = (uint8_t*) &ADDR_PIC;
	uint8_t * tmpr2 = (uint8_t*) &command;

//	lcd_setup_picture(pic_nr);
	ADDR_PIC = pic_nr * (600*256);

	i2 = 3;
    for(i=0; i<3; i++){
    	tmpr2[i2] = tmpr1[i];
    	i2--;
    }

	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);

	HAL_SPI_Transmit(&hspi1,  tmpr2, 4, 5555);

	for(i = 0; i < 600; i++) {
		HAL_SPI_Receive(&hspi1, &page_data[0], 256, 5555);
		for(i2 = 0; i2 < 128; i2++){
			LCD_WriteData(DATA_PAGE[i2]);
		}
	}

	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(5);

  return 0;
}

//=====================================================================================================================

/**
  * @brief  Returns the name of the day of the week.
  *         This function takes a number (1-7) and returns the corresponding
  *         day of the week as a string.
  * @param  number: The day of the week as a number (1 for Monday, 2 for Tuesday, etc.).
  * @retval char*: The name of the corresponding day of the week.
  */
char* getDayofweek(uint8_t number)
{
	if(number == 0) { number = 1;}
	if(number  > 7) { number = 7;}
	char *weekday[] = { "MONDAY", "TUESDAY", "WEDNESDAY","THURSDAY","FRIDAY","SATURDAY" , "SUNDAY"};
	return weekday[number-1];
}
//=====================================================================================================================
int calc_dow(int y, int m,int d) {
	int result;

	y-=m<3;
	result = (y+y/4-y/100+y/400+"-bed=pen+mad."[m]+d)%7;
	if(result == 0) {result = 7;}
	return result;
}
//=====================================================================================================================
void set_RTC_DATE(void){
	check_RTC_DATE();
	HAL_RTC_SetDate(&hrtc, &my_date, RTC_FORMAT_BIN);
}
void check_RTC_DATE(void){
	static const uint8_t month_maxdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31} ;
	uint8_t luty_add = 0;

	if(my_date.Year > 99) { my_date.Year = my_date.Year % 100;}
	if((my_date.Year %4) == 0) {luty_add =1;}
	if(my_date.Month == 0) {my_date.Month =1;}
	if(my_date.Month > 12) {my_date.Month =12;}
	max_day = month_maxdays[my_date.Month - 1];
	if(my_date.Month == 2) { max_day = max_day + luty_add;}
	if(my_date.Date == 0 ) {my_date.Date = 1;}
	if(my_date.Date > max_day) {my_date.Date = max_day;}
	my_date.WeekDay = calc_dow((int) (((century - 1) * 100) + my_date.Year)  , (int) my_date.Month , (int) my_date.Date );


}
//=====================================================================================================================

//=====================================================================================================================
void colon_print(void){
		TIM4->CR1 = TIM4->CR1 | 1;
		HAL_TIM_Base_Start_IT(&htim4);
		LCD_DisASquare(158, 36, 12, lcd_text_color);
		LCD_DisASquare(158, 78, 12, lcd_text_color);
}
//=====================================================================================================================
void read_RTC(void){
	  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	  HAL_RTC_WaitForSynchro(&hrtc);
	  HAL_RTC_GetTime(&hrtc, &my_time, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &my_date, RTC_FORMAT_BIN);
}
//=====================================================================================================================
void check_night_mode(void) {

#define nigh_value 20

	int hh;
	read_RTC();
	hh = my_time.Hours;

	if((mode_night &1) == 0) {
		if(hh >= nigh_value) {
			set_night_PWM();
			return;
		}
		if(hh < 6)  {
			set_night_PWM();
			return;
		}
	}
	set_PWM(); // out of night mode turn on the light as day mode

}
//=====================================================================================================================
void print_RTC(void) {

	lcdSetOrientation(LCD_WORK_ORIENTATION);
//	LCD_ClrScr(lcd_background_color);
	check_night_mode();
	int hh = my_time.Hours, mm = my_time.Minutes, ss = my_time.Seconds;



	set_color_RTC();

	old_time.Seconds = my_time.Seconds;
	sprintf(tekst, "%02d", hh);
	lcd_mono_text_boxed(2, 0, tekst, digital_7_ttf, 86);

	sprintf(tekst, "%02d", mm);
	lcd_mono_text_boxed(167, 0, tekst, digital_7_ttf, 86);
	sprintf(tekst, "%02d", ss);
	lcd_mono_text_boxed(0, 125, tekst, digital_7_ttf, 40);


	LCD_DisARectangular(200, 125, 319, 185, lcd_background_color); // clear for short name
	lcd_text_boxed(88, 140, getDayofweek(my_date.WeekDay), dum1_ttf, 42);

	hh = my_date.Year + ((century - 1) * 100);
	sprintf(tekst, "%04d-%02d-%02d", hh, my_date.Month, my_date.Date);
	if(century > -10) {
		lcd_mono_text_boxed(5, 185, tekst, digital_7_ttf, 36);
	} else {
		lcd_mono_text_boxed(5, 185, tekst, digital_7_ttf, 30);
	}

	colon_print();
}
//=====================================================================================================================
void set_color_RTC(void)
{
  lcd_text_color =  0xffff ;
  lcd_background_color = 0x0000;

  if ((change_colors & 0x01)!= 0) { return;}

  if(my_time.Minutes < 15) {
	  lcd_text_color = COLOR_565_GREEN;
	  return;
  }
  if(my_time.Minutes < 30) {
	  lcd_text_color = COLOR_565_RED;
	  return;
  }
  if(my_time.Minutes < 45) {
	  lcd_text_color = COLOR_565_MAGENTA;
	  return;
  }
}

//=====================================================================================================================
void test_print_RTC(void){
	int32_t tmpr1;
	uint8_t tmpr2, tmpr3;
	struct tm * czas_lokal;
	//---
	read_RTC();
	tmpr2 = my_time.Minutes;

	int ss = my_time.Seconds;
	set_color_RTC();
	if(my_time.Seconds != old_time.Seconds) {
		old_time.Seconds = my_time.Seconds;


		if(ss == 3) {
			if(UPDATE_FLAG == 0){ compare_EPOCH(); }
		}
		//TODO ADD TIME
		if(UPDATE_FLAG == 1) {
			if(my_time.Seconds == 4) {

				UPDATE_FLAG = 0; // clear update flag
				tmpr1 = SECONDS_TO_CALIBRATE; // get for compare and calculations seconds calibrate value

//------------------
				if(tmpr1 != 0) { // exist calibration seconds value != 0 ?

					read_timestamp();
					timestamp = timestamp + tmpr1;
					czas_lokal = gmtime(&timestamp);

					my_date.Year     =  czas_lokal->tm_year % 100 ;
					my_date.Date     =  czas_lokal->tm_mday ;
					my_date.Month    =  czas_lokal->tm_mon +1 ;
					my_date.WeekDay  =  czas_lokal->tm_mday ;

					my_time.Hours    =  czas_lokal->tm_hour ;
					my_time.Minutes  =  czas_lokal->tm_min ;
					my_time.Seconds  =  czas_lokal->tm_sec ;

					set_RTC_DATE();
					HAL_RTC_SetTime(&hrtc, &my_time, RTC_FORMAT_BIN); // store setup value
					ss = my_time.Seconds;
					print_RTC(); // immediatelly RTC print
//---
				}
			}
		}

		if(my_time.Seconds == 0) {
			print_RTC();
			return;
		}


		//TODO image
		sprintf(tekst, "%02d", ss);
		lcd_mono_text_boxed(0, 125, tekst, digital_7_ttf, 40);
		if(ss == 25) {
			if(run_slideshow == 0) {
				if(tmpr2 %5 == 0) {

					tmpr3 = tmpr2 / 5;
					TIM4_IRQHandler(); // stop colon blinking
					lcd_setup_picture(tmpr3);
					readPicFromFlash(tmpr3);
					HAL_Delay(10000);
					lcdSetOrientation(LCD_WORK_ORIENTATION);
					LCD_ClrScr(lcd_background_color);

					print_RTC();
					return;
				}
			}
		}
		colon_print();
	}
}
//=====================================================================================================================
//=====================================================================================================================




//=====================================================================================================================
bool setting_globals(void) {

	if((test_pressed_point(70, 319, 125, 185)) == true)  {
		TIM4_IRQHandler(); // disable timer4 and IRQ
		LCD_ClrScr(lcd_background_color);

		while(1){

			lcd_text_boxed(0, 0, "GLOBAL FUNCTIONS", dum1_ttf, 36);

			lcd_text_boxed(25, 50, "PAINT", dum1_ttf, 42);
			lcd_text_boxed(175, 50, "BIGTTF", dum1_ttf, 42);
			lcd_text_boxed(25, 100, "LIGHT", dum1_ttf, 42);
			lcd_text_boxed(165,100, "COLORS", dum1_ttf, 42);
			lcd_text_boxed(22, 150, "AUTO", dum1_ttf, 42);
			lcd_text_boxed(170,150, "CALIBR.", dum1_ttf, 42);
			lcd_text_boxed(22,200, "EXIT", dum1_ttf, 42);
			lcd_text_boxed(190,200, "NEXT", dum1_ttf, 42);

			LCD_No_Fill_Draw(0xffff, 0, 45, 159, 95); // paint if(test_pressed_point(0, 159, 45, 95) == true)
			LCD_No_Fill_Draw(0xffff, 159, 45, 319, 95); // ttf demo if(test_pressed_point(159, 319, 45, 95) == true)
			LCD_No_Fill_Draw(0xffff, 0, 95, 159, 145); // light if((test_pressed_point(0, 159, 95, 145)) == true)
			LCD_No_Fill_Draw(0xffff, 159, 95, 319, 145); // colors if((test_pressed_point(159, 319, 95, 145)) == true)
			LCD_No_Fill_Draw(0xffff, 0, 145, 159, 195); // auto if((test_pressed_point(0, 159, 145, 195)) == true)

			LCD_No_Fill_Draw(0xffff, 159, 145, 319, 195); // TIM5 CALIBR  if((test_pressed_point(159, 319, 145, 195)) == true)

			LCD_No_Fill_Draw(0xffff, 0, 195, 159, 239);  // EXIT  if((test_pressed_point(0, 159, 195, 239)) == true)
			LCD_No_Fill_Draw(0xffff, 159, 195, 319, 239); // NEXT  if((test_pressed_point(159, 319, 195, 239)) == true)

			get_pressed_point();
// TODO GLOBALS
			TIM_CALIBRATE_SETUP();
			paint_test();
			TTF_test();
			set_LIGHT();
			COLOR_FONTS();
			night_mode();
			if( go_to_back() == true) { return true;}
			test_page();
		}
	}
	return false; // doesn't choose setting_globals
}
//=====================================================================================================================

//=====================================================================================================================
void next_page(void) {


	while(1){

		lcd_text_boxed(10, 0, "OTHER FUNCTIONS", dum1_ttf, 36);

		lcd_text_boxed(3, 50, "CAL_TS", dum1_ttf, 42);
		lcd_text_boxed(175, 50, "SLIDES", dum1_ttf, 42);

					lcd_text_boxed(10, 100, "DEMOS", dum1_ttf, 42);
					lcd_text_boxed(165,100, "V_BALL", dum1_ttf, 42);
					lcd_text_boxed(5, 155, "ROTATE", dum1_ttf, 38);
					lcd_text_boxed(162,154, "CENTURY", dum1_ttf, 37);
					lcd_text_boxed(22,200, "EXIT", dum1_ttf, 42);
					lcd_text_boxed(190,200, "PICS", dum1_ttf, 42);

		// TODO place where try obtains press parameter area
		LCD_No_Fill_Draw(0xffff, 0, 45, 159, 95); // CAL_TS if(test_pressed_point(0, 159, 45, 95) == true)
		LCD_No_Fill_Draw(0xffff, 159, 45, 319, 95); // SLIDES demo if(test_pressed_point(159, 319, 45, 95) == true)
		LCD_No_Fill_Draw(0xffff, 0, 95, 159, 145); // DEMOS if((test_pressed_point(0, 159, 95, 145)) == true)
		LCD_No_Fill_Draw(0xffff, 159, 95, 319, 145); // BALL  if((test_pressed_point(159, 319, 95, 145)) == true)
		LCD_No_Fill_Draw(0xffff, 0, 145, 159, 195); // ROTATION if((test_pressed_point(40, 280, 145, 195)) == true)

		LCD_No_Fill_Draw(0xffff, 159, 145, 319, 195); // CENTURY  if((test_pressed_point(159, 319, 145, 195)) == true)

		LCD_No_Fill_Draw(0xffff, 0, 195, 159, 239);  // EXIT  if((test_pressed_point(0, 159, 195, 239)) == true)
		LCD_No_Fill_Draw(0xffff, 159, 195, 319, 239); // NEXT  if((test_pressed_point(159, 319, 195, 239)) == true)


// TODO NEXT_PAGE
		get_pressed_point();
		CAL_TS();
		SLIDE_SHOW();
		DEMOS_RUN();
		V_BALL();
		ROTATION_TEST();
		slideshow_disable();
		SET_CENTURY();
		if( go_to_back() == true) { return ; }



	}
}
//=====================================================================================================================


//============================
// TODO LIST: ADD FUNC TO LOOP
// -
//============================
//=====================================================================================================================



//=====================================================================================================================
// TODO: ACTUAL FUNCTION MAKING
//=====================================================================================================================
//=====================================================================================================================

//=====================================================================================================================
void SET_CENTURY(void) {

	uint8_t action ;
	read_RTC();
	READ_CENTURY();


	if((test_pressed_point(159, 319, 145, 195)) == true)  {

		LCD_ClrScr(lcd_background_color);
		while(1){
			lcd_text_boxed(55, 5, "SET CENTURY", dum1_ttf, 42);
//===================================================================================================
			lcd_mono_text_boxed(220, 50, "+", digital_7_ttf, 86);
			lcd_mono_text_boxed(0, 50, "-", digital_7_ttf, 86);
			sprintf(tekst, "%02d", (int) century);
			lcd_mono_text_boxed(80, 50, "  ", digital_7_ttf, 86);

			if(century < 0) {
				lcd_mono_text_boxed(80, 78, tekst, digital_7_ttf, 50);
			} else {
				lcd_mono_text_boxed(123, 78, tekst, digital_7_ttf, 50);
			}
			lcd_text_boxed(115, 180, "DONE", dum1_ttf, 42);
			LCD_No_Fill_Draw(lcd_text_color, 80, 172, 240, 230);
			LCD_No_Fill_Draw(lcd_text_color, 0, 55, 80, 165);
			LCD_No_Fill_Draw(lcd_text_color, 235, 55, 318, 165);
//===================================================================================================
			check_RTC_DATE();
			action = test_action();
			if((action == 3)) { // DONE pressed

				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR10,  (uint32_t) century);
				read_RTC();
				set_RTC_DATE();
				LCD_ClrScr(lcd_background_color);
				HAL_Delay(100);
				return ;
			} else {
				if(action == 2) { // PLUS pressed
					if(century == 99) { century = -99;} else { century++;}
				} else { // MINUS pressed
					if(century == -99) { century = 99;} else { century--;}
				}
			}
		}
	}
}

//=====================================================================================================================
void READ_CENTURY(void) {
	century = (int32_t) HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR10);
	if(century > 99) {
		century = 21;
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR10,  (uint32_t) century);
	}
	if(century < -99) {
		century = 21;
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR10,  (uint32_t) century);
	}
}
//=====================================================================================================================




//=====================================================================================================================
void compare_EPOCH() {
	uint64_t tmpr01;
	time_t pgm_timestamp;
	double elapsed;
	uint32_t hours, tmpr1, tmpr_hi, tmpr_lo;

	tmpr01 = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR9); // read high 32 bits
	tmpr01 = tmpr01 << 32;
	tmpr01 = tmpr01 |  HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR8); // add low 32 bits

	pgm_timestamp = (time_t) tmpr01;
	read_timestamp(); // read RTC to timestamp
	elapsed = difftime(timestamp, pgm_timestamp); // = TIMESTAMP - PGM_TIMESTAMP

	// ELAPSED = ACTUAL TIME - PGM_TIMESTAMP
	if(elapsed > 0) {
		UPDATE_FLAG = 1;
		hours = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4); // iterval hours calibrate event

		tmpr1 = hours * 3600;


		timestamp = pgm_timestamp;
		timestamp = timestamp + tmpr1;

		tmpr_hi = (uint32_t) (timestamp >> 32);
		tmpr_lo = (uint32_t) (timestamp & 0xFFFFFFFF);

		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR8,  tmpr_lo);  // LO timestamp
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR9, tmpr_hi);  // HI timestamp

	}
}

//=====================================================================================================================
void set_EPOCH(void) { // setting the EPOCH date to calibrate seconds date
	uint32_t hours, tmpr1, tmpr_hi, tmpr_lo;

	hours = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4); // iterval hours calibrate event

	tmpr1 = hours * 3600;

	read_timestamp(); // read RTC to timestamp

	timestamp = timestamp + tmpr1;

	tmpr_hi = (uint32_t) (timestamp >> 32);
	tmpr_lo = (uint32_t) (timestamp & 0xFFFFFFFF);

	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR8,  tmpr_lo);  // LO timestamp
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR9, tmpr_hi);  // HI timestamp

	UPDATE_FLAG = 0;
}
//=====================================================================================================================
void read_timestamp(void) {
	HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);

	currTime.tm_year = currentDate.Year + ((century -20) * 100);  // In fact: 2000 + 18 - 1900
	currTime.tm_mday = currentDate.Date;
	currTime.tm_mon  = currentDate.Month - 1;

	currTime.tm_hour = currentTime.Hours;
	currTime.tm_min  = currentTime.Minutes;
	currTime.tm_sec  = currentTime.Seconds;

	timestamp = mktime(&currTime);
}
//=====================================================================================================================
void night_light(void){
	uint8_t action;
	uint32_t LIGHT_VAL = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR7);
	if(LIGHT_VAL > 10) {LIGHT_VAL = 10;}

	set_night_PWM();
	LCD_ClrScr(lcd_background_color);

	while(1){
		lcd_text_boxed(85, 5, "SET NIGHT", dum1_ttf, 42);
		sprintf(tekst, "%02d", (int) LIGHT_VAL);
		lcd_mono_text_boxed(125, 85, tekst, digital_7_ttf, 36);
		put_DATE_number();
		action = test_action();
		set_night_PWM();

		Pressed_Point.x =0;
		Pressed_Point.y =0;
		if((action == 3)) { // DONE pressed
			wait_for_releasseTS(); // wait for releasse TS
			LCD_ClrScr(lcd_background_color);
			check_night_mode();
			return ;
		} else {
			if(action == 2) { // PLUS pressed
				if(LIGHT_VAL < 10) { LIGHT_VAL++;}
			} else { // MINUS pressed
				if(LIGHT_VAL >  0)  { LIGHT_VAL--;}
			}
			HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR7, LIGHT_VAL);
			set_night_PWM();
		}

	}




}
//=====================================================================================================================
void slideshow_disable(void){
	uint8_t s_mode, action;
	if((test_pressed_point(159, 319, 195, 239)) == true){
		LCD_ClrScr(lcd_background_color);
		check_slideshow(); // read slide setup
		s_mode = run_slideshow;

		while(1){

			s_mode = s_mode &0x01;

			lcd_text_boxed(25, 5, "SLIDE MODE", dum1_ttf, 42);
			sprintf(tekst, "YES");
			if(s_mode  != 0) { sprintf(tekst, "NO!"); }

			lcd_mono_text_boxed(105, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();

			Pressed_Point.x =0;
			Pressed_Point.y =0;
			if((action == 3)) { // DONE pressed
				read_RTC();
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR6, (uint32_t) s_mode);
				run_slideshow = s_mode ; // read back data from backup register to variable

				wait_for_releasseTS(); // wait for releasse TS
				LCD_ClrScr(lcd_background_color);
				return ;
			} else {
				wait_for_releasseTS(); // wait for releasse TS
				if(action == 2) { // PLUS pressed
					s_mode++;
				} else { // MINUS pressed
					s_mode--;
				}
			}

		}
	}
}
//=====================================================================================================================
void check_slideshow() {
	run_slideshow = (uint8_t) (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR6) & 0x01 );
}
//=====================================================================================================================
void ROTATION_TEST(void) {
	uint8_t action;
	uint32_t R_MODE;



	if((test_pressed_point(0, 159, 145, 195)) == true)  {
		LCD_ClrScr(lcd_background_color);
		R_MODE= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
		if(R_MODE != 0) { R_MODE = 1;}


		while(1){



			R_MODE = R_MODE & 0x01;

			if(R_MODE != 0)
			{
				LCD_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE_ROTATE ;
			} else {
				LCD_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE;
			}
			HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, LCD_WORK_ORIENTATION);
			check_work_orientation();
			lcdSetOrientation(LCD_WORK_ORIENTATION);
			LCD_ClrScr(lcd_background_color);

			lcd_text_boxed(25, 5, "SET ROTATION", dum1_ttf, 42);
			sprintf(tekst, " 0 ");
			if(R_MODE != 0) { sprintf(tekst, "180"); }

			lcd_mono_text_boxed(105, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();

			Pressed_Point.x =0;
			Pressed_Point.y =0;
			if((action == 3)) { // DONE pressed

				wait_for_releasseTS(); // wait for releasse TS
				LCD_ClrScr(lcd_background_color);

				return ;
			} else {
				wait_for_releasseTS(); // wait for releasse TS
				if(action == 2) { // PLUS pressed
					R_MODE++;
				} else { // MINUS pressed
					R_MODE--;
				}
			}

		}
	}
}
//=====================================================================================================================
void check_work_orientation(void){
	LCD_WORK_ORIENTATION = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
	if((LCD_WORK_ORIENTATION &3) != 0) {
		LCD_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE_ROTATE;
		LCD_NOT_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE ;
		LCD_PORTRAIT_WORK_ORIENTATION = LCD_ORIENTATION_PORTRAIT_ROTATE_MIRROR;
		LCD_PORTRAIT_NOT_WORK_ORIENTATION = LCD_ORIENTATION_PORTRAIT;
	} else {
		LCD_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE;
		LCD_NOT_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE_ROTATE ;
		LCD_PORTRAIT_WORK_ORIENTATION = LCD_ORIENTATION_PORTRAIT;
		LCD_PORTRAIT_NOT_WORK_ORIENTATION = LCD_ORIENTATION_PORTRAIT_ROTATE_MIRROR;
	}
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, LCD_WORK_ORIENTATION);
}
//=====================================================================================================================
void V_BALL(void) {

	if((test_pressed_point(159, 319, 95, 145)) == true) {
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 65535);

		lcdSetOrientation(LCD_ORIENTATION_PORTRAIT);
		main_app();

		set_PWM();
		lcdSetOrientation(LCD_WORK_ORIENTATION);
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
		Pressed_Point.x =0;
		Pressed_Point.y =0;
	}
}
//=====================================================================================================================
void DEMOS_RUN(void) {

	if((test_pressed_point(0, 159, 95, 145)) == true) {
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 65535);

		start_demos();
		set_PWM();
		lcdSetOrientation(LCD_WORK_ORIENTATION);
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
		Pressed_Point.x =0;
		Pressed_Point.y =0;
	}
}
//=====================================================================================================================


//=====================================================================================================================
void SLIDE_SHOW(void) {
	uint8_t i;

	if(test_pressed_point(159, 319, 45, 95) == true)  {
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);

		for( i=0; i<12; i++) {
			lcd_setup_picture(i);
			readPicFromFlash(i);
			get_pressed_point();
			wait_for_releasseTS(); // wait for releasse TS
		}
		lcdSetOrientation(LCD_WORK_ORIENTATION);

		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
		Pressed_Point.x =0;
		Pressed_Point.y =0;
	}
}
//=====================================================================================================================

//=====================================================================================================================
void CAL_TS(void) {
	if(test_pressed_point(0, 159, 45, 95) == true)  {
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 65535);
		lets_calibrate_ts(LCD_ORIENTATION_LANDSCAPE_ROTATE);
		LCD_ClrScr(lcd_background_color);
		Paint_Color = COLOR_565_WHITE;
		start_Paint();

		set_PWM();
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
	}
}
//=====================================================================================================================
void test_page(void) {
	if((test_pressed_point(159, 319, 195, 239)) == true)  {
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);

		next_page();

		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
	}
}
//=====================================================================================================================
void TIM_CALIBRATE_SETUP(void) {
	uint8_t action ;
	if((test_pressed_point(159, 319, 145, 195)) == true)  {
		TIM4_IRQHandler(); // disable timer4 and IRQ
		LCD_ClrScr(lcd_background_color);


		while(1){
			lcd_text_boxed(0, 5, "SET TIM CALIBR.", dum1_ttf, 42);

			sprintf(tekst, "   ");
			lcd_mono_text_boxed(130, 55, tekst, digital_7_ttf, 36); // clear field by spaces
			sprintf(tekst, "%d", ((int)SECONDS_TO_CALIBRATE));

			if(SECONDS_TO_CALIBRATE < 0) {
				lcd_mono_text_boxed(130, 55, tekst, digital_7_ttf, 36); // print seconds
			} else {
				lcd_mono_text_boxed(145, 55, tekst, digital_7_ttf, 36); // print seconds
			}
			sprintf(tekst, "%3d", ((int)HOURS_CALIBRATE_PERIOD));
			lcd_mono_text_boxed(125, 110, tekst, digital_7_ttf, 36); // print seconds
//==================================================================
			lcd_mono_text_boxed(260, 55, "+", digital_7_ttf, 36);
			lcd_mono_text_boxed(20, 55, "-", digital_7_ttf, 36);
			lcd_mono_text_boxed(260, 110, "+", digital_7_ttf, 36);
			lcd_mono_text_boxed(20, 110, "-", digital_7_ttf, 36);

			lcd_text_boxed(85, 68, "SEC", dum1_ttf, 28);
			lcd_text_boxed(85, 123, "HRS", dum1_ttf, 28);

			lcd_text_boxed(115, 180, "DONE", dum1_ttf, 42);
			LCD_No_Fill_Draw(lcd_text_color, 80, 172, 240, 230); // DONE


			LCD_No_Fill_Draw(lcd_text_color, 0, 55, 80, 110); // minus sec
			LCD_No_Fill_Draw(lcd_text_color, 0, 110, 80, 165); // minus hrs

			LCD_No_Fill_Draw(lcd_text_color, 235, 55, 318, 110); // plus sec
			LCD_No_Fill_Draw(lcd_text_color, 235, 110, 318, 165); // plus hrs
//===================================================================

			action = calibr_action();

			Pressed_Point.x =0;
			Pressed_Point.y =0;
//TODO EPOCH
			if((action == 5)) { // DONE pressed

				wait_for_releasseTS(); // wait for releasse TS
				LCD_ClrScr(lcd_background_color);
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, HOURS_CALIBRATE_PERIOD);
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, (uint32_t) SECONDS_TO_CALIBRATE);
				set_EPOCH();

				return ;
			} else {
				if(action == 2) { // PLUS sec pressed
					if(SECONDS_TO_CALIBRATE < 99)  { SECONDS_TO_CALIBRATE++;} else { SECONDS_TO_CALIBRATE = -99;}
				}
				if (action == 1) { // MINUS sec pressed
					if(SECONDS_TO_CALIBRATE > -99)  { SECONDS_TO_CALIBRATE--;} else { SECONDS_TO_CALIBRATE = 99;}
				}
				if(action == 4) { // PLUS hrs pressed
					if(HOURS_CALIBRATE_PERIOD < 744)  { HOURS_CALIBRATE_PERIOD++;} else { HOURS_CALIBRATE_PERIOD = 1 ;}
				}
				if(action == 3) { // PLUS hrs pressed
					if(HOURS_CALIBRATE_PERIOD > 1)  { HOURS_CALIBRATE_PERIOD--;} else { HOURS_CALIBRATE_PERIOD = 744 ;}
				}
			}

		}

	}

}
//=====================================================================================================================
uint8_t calibr_action(void) {
	while(1) {
		get_pressed_point();
		// DONE test
		if((test_pressed_point(80, 240, 172, 230)) == true) { return 5;} // DONE

		if((test_pressed_point(0, 80, 55, 110)) == true) { return 1;} // minus sec
		if((test_pressed_point(0, 80, 110, 165)) == true) { return 3;} // minus hrs

		if((test_pressed_point(235, 318, 55, 110)) == true) { return 2;} // plus sec
		if((test_pressed_point(235, 318, 110, 165)) == true) { return 4;} // plus hrs

	}
}
//=====================================================================================================================
void CHECK_CALIBRATE_TIME_VALUES(void){
	uint32_t hours ;
	int32_t  seconds;
	hours = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4); // iterval hours calibrate event
	seconds = (int32_t) HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR5); // seconds to add calibrate

	if(hours == 0)      { hours   = 1;   }
	if(hours > 744)     { hours   = 744; }
	if(seconds > 99)    { seconds = 0;   } // seconds to calibrate
	if(seconds < (-99)) { seconds = 0;   } // seconds to calibrate

	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, (uint32_t) seconds);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, hours);
	HOURS_CALIBRATE_PERIOD = hours;
	SECONDS_TO_CALIBRATE   = seconds;
}
//=====================================================================================================================
bool go_to_back(void) {
	if((test_pressed_point(0, 159, 195, 239)) == true)  {
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
		HAL_Delay(100);
		return true;
	}
	return false; // doesn't choose set year
}
//=====================================================================================================================
void night_mode(void) {
	uint8_t action;
	uint32_t n_mode;



	if((test_pressed_point(0, 159, 145, 195)) == true)  {
		LCD_ClrScr(lcd_background_color);
		n_mode = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);

		while(1){

			n_mode = n_mode &0x01;

			lcd_text_boxed(25, 5, "NIGHT MODE", dum1_ttf, 42);
			sprintf(tekst, "YES");
			if(n_mode  != 0) { sprintf(tekst, "NO!"); }

			lcd_mono_text_boxed(105, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();

			Pressed_Point.x =0;
			Pressed_Point.y =0;
			if((action == 3)) { // DONE pressed
				read_RTC();
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, n_mode);
				mode_night = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3); // read back data from backup register to variable

				wait_for_releasseTS(); // wait for releasse TS
				LCD_ClrScr(lcd_background_color);
				if(mode_night == 0) {night_light();}
				check_night_mode();
				return ;
			} else {
				wait_for_releasseTS(); // wait for releasse TS
				if(action == 2) { // PLUS pressed
					n_mode++;
				} else { // MINUS pressed
					n_mode--;
				}
			}

		}

	}

}
//=====================================================================================================================
void COLOR_FONTS(void) {
	uint8_t action;
	uint32_t font_VAL = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);


	if((test_pressed_point(159, 319, 95, 145)) == true)  {
		LCD_ClrScr(lcd_background_color);

		while(1){
			lcd_text_boxed(25, 5, "COLOR FONTS", dum1_ttf, 42);
			sprintf(tekst, "YES");
			if((font_VAL &1) != 0) { sprintf(tekst, "NO!"); }
			lcd_mono_text_boxed(105, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();

			Pressed_Point.x =0;
			Pressed_Point.y =0;
			if((action == 3)) { // DONE pressed
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, font_VAL);
				change_colors = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
				wait_for_releasseTS(); // wait for releasse TS
				LCD_ClrScr(lcd_background_color);
				HAL_Delay(100);
				return ;
			} else {
				if(action == 2) { // PLUS pressed
					font_VAL++;
				} else { // MINUS pressed
					font_VAL--;
				}
			}
		}
	}
}
//=====================================================================================================================

//=====================================================================================================================
void set_LIGHT(void) {
	uint8_t action;
	uint32_t LIGHT_VAL = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	if(LIGHT_VAL > 10) {LIGHT_VAL = 10;}

	if((test_pressed_point(0, 159, 95, 145)) == true)  {
		LCD_ClrScr(lcd_background_color);

		while(1){
			lcd_text_boxed(85, 5, "SET LIGHT", dum1_ttf, 42);
			sprintf(tekst, "%02d", (int) LIGHT_VAL);
			lcd_mono_text_boxed(125, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();

			Pressed_Point.x =0;
			Pressed_Point.y =0;
			if((action == 3)) { // DONE pressed
				wait_for_releasseTS(); // wait for releasse TS
				HAL_Delay(100);
				LCD_ClrScr(lcd_background_color);
				return ;
			} else {
				if(action == 2) { // PLUS pressed
					if(LIGHT_VAL < 10) { LIGHT_VAL++;}
				} else { // MINUS pressed
					if(LIGHT_VAL >  0)  { LIGHT_VAL--;}
				}
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, LIGHT_VAL);
				set_PWM();
			}

		}

	}
}
//=====================================================================================================================
void set_PWM(void) {
	uint16_t light;
	uint32_t light_val = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	if(light_val == 0) { light = 4096;}
	if(light_val > 10) { light_val = 10;}
	if(light_val > 0)  { light = light_val * 6553; }
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, light_val);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, light);
}

void set_night_PWM(void) {
	uint16_t light;
	uint32_t light_val = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR7);
	if(light_val == 0) { light = 4096;}
	if(light_val > 10) { light_val = 10;}
	if(light_val > 0)  { light = light_val * 6553; }
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR7, light_val);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, light);
}
//=====================================================================================================================
void TTF_test(void) {
	extern uint16_t Paint_Color;

	if(test_pressed_point(159, 319, 45, 95) == true)  {
		Paint_Color = COLOR_565_WHITE;
		Big_TTF_Demo();
		wait_for_releasseTS(); // wait for releasse TS
		Pressed_Point.x =0;
		Pressed_Point.y =0;
	}
}
//=====================================================================================================================

//=====================================================================================================================
void paint_test(void) {
	if(test_pressed_point(0, 159, 45, 95) == true)  {
		Paint_Color = COLOR_565_WHITE;
		start_Paint();
		wait_for_releasseTS(); // wait for releasse TS
		Pressed_Point.x =0;
		Pressed_Point.y =0;
	}
}
//=====================================================================================================================


//=====================================================================================================================


//=====================================================================================================================



//=====================================================================================================================
bool set_day(void) {
	uint8_t action;
	if((test_pressed_point(230, 319, 185, 239)) == true)  {
		TIM4_IRQHandler(); // disable timer4 and IRQ
		LCD_ClrScr(lcd_background_color);
		check_RTC_DATE(); // get max day in month
		while(1){
			lcd_text_boxed(85, 5, "SET DAY", dum1_ttf, 42);
			sprintf(tekst, "%02d", (my_date.Date));
			lcd_mono_text_boxed(125, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();
			check_RTC_DATE();
			if((action == 3)) { // DONE pressed
				set_RTC_DATE();
				LCD_ClrScr(lcd_background_color);
				print_RTC();
				return true;
			} else {
				if(action == 2) { // PLUS pressed
					if(my_date.Date == max_day) { my_date.Date = 1;} else { my_date.Date++;}
				} else { // MINUS pressed
					if(my_date.Date == 1) { my_date.Date = max_day;} else { my_date.Date--;}
				}
			}

		}

	}
	return false; // doesn't choose set year
}
//=====================================================================================================================
bool set_month(void) {
	uint8_t action;
	if((test_pressed_point(130, 230, 185, 239)) == true)  {
		TIM4_IRQHandler(); // disable timer4 and IRQ
		LCD_ClrScr(lcd_background_color);
		while(1){
			lcd_text_boxed(55, 5, "SET MONTH", dum1_ttf, 42);
			sprintf(tekst, "%02d", (my_date.Month));
			lcd_mono_text_boxed(125, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();
			check_RTC_DATE();
			if((action == 3)) { // DONE pressed
				set_RTC_DATE();
				LCD_ClrScr(lcd_background_color);
				print_RTC();
				return true;
			} else {
				if(action == 2) { // PLUS pressed
					if(my_date.Month == 12) { my_date.Month = 1;} else { my_date.Month++;}
				} else { // MINUS pressed
					if(my_date.Month == 1) { my_date.Month = 12;} else { my_date.Month--;}
				}
			}

		}

	}
	return false; // doesn't choose set year
}
//=====================================================================================================================
bool set_year(void) {
	uint8_t action;
	if((test_pressed_point(0, 130, 185, 239)) == true)  {
		TIM4_IRQHandler(); // disable timer4 and IRQ
		LCD_ClrScr(lcd_background_color);
		while(1){
			lcd_text_boxed(65, 5, "SET YEAR", dum1_ttf, 42);
			sprintf(tekst, "%04d", (int) (my_date.Year + ((century - 1) * 100)));
			lcd_mono_text_boxed(95, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();
			check_RTC_DATE();
			if((action == 3)) { // DONE pressed
				set_RTC_DATE();
				LCD_ClrScr(lcd_background_color);
				print_RTC();
				return true;
			} else {
				if(action == 2) { // PLUS pressed
					if(my_date.Year == 99) { my_date.Year = 0;} else { my_date.Year++;}
				} else { // MINUS pressed
					if(my_date.Year == 0) { my_date.Year = 99;} else { my_date.Year--;}
				}
			}

		}

	}
	return false; // doesn't choose set year
}
//=====================================================================================================================
void get_pressed_point(void){
	POINT OneSample;
	while(XPT2046_TouchPressed() != true);
	while((XPT2046_GetFastCoordinates(&OneSample.x, &OneSample.y)) != true ){ ; }
	getDisplayPoint( &Pressed_Point, &OneSample, &matrix ) ;
	if(LCD_WORK_ORIENTATION == LCD_ORIENTATION_LANDSCAPE) {
		Pressed_Point.x = 319 - Pressed_Point.x;
		Pressed_Point.y = 239 - Pressed_Point.y;
	}
	my_utoa(&idx[0], Pressed_Point.x);
	my_utoa(&idy[0], Pressed_Point.y);
	wait_for_releasseTS(); // wait for releasse TS
	HAL_Delay(20);
}
//=====================================================================================================================
void test_setup(void)
{

	if((XPT2046_TouchPressed()) == false) { return; } // no touch pressed so good bye

	get_pressed_point();  // get pressed points coords

	if(clear_ss() == true) { return; } // clear seconds test
	lcd_text_color = 0xFFFF;
	// todo KAFAR
	if(set_hours() == true) { return; }
	if(set_minuts() == true) { return; }
	if(set_year() == true) { return; }
	if(set_month() == true) { return; }
	if(set_day() == true) { return; }
	if(setting_globals() == true) {
		print_RTC();
		wait_for_releasseTS(); // wait for releasse TS
		return;
	}

	wait_for_releasseTS(); // wait for releasse TS if pressed
}
//=====================================================================================================================
bool set_minuts(void) {

	uint8_t action , bckp, chk_min;
	read_RTC();
	chk_min = my_time.Minutes;
	if((test_pressed_point(180, 319, 0, 115)) == true)  {
		TIM4_IRQHandler(); // disable timer4 and IRQ
		LCD_ClrScr(lcd_background_color);
		while(1){
			lcd_text_boxed(35, 5, "SET MINUTS", dum1_ttf, 42);
			put_number( my_time.Minutes);
			action = test_action();
			if((action == 3)) { // DONE pressed
				bckp = my_time.Minutes;
				read_RTC();
				my_time.Minutes = bckp;

				if(chk_min != my_time.Minutes) {

					my_time.Seconds ++;
					if(my_time.Seconds > 59) { my_time.Seconds = 0;}
					__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
					HAL_RTC_WaitForSynchro(&hrtc);
					HAL_RTC_SetTime(&hrtc, &my_time, RTC_FORMAT_BIN); // store setup value
				}

				LCD_ClrScr(lcd_background_color);
				print_RTC();
				return true;
			} else {
				if(action == 2) { // PLUS pressed
					if(my_time.Minutes == 59) { my_time.Minutes = 0;} else { my_time.Minutes++;}
				} else { // MINUS pressed
					if(my_time.Minutes == 0) { my_time.Minutes = 59;} else { my_time.Minutes--;}
				}
			}

		}

	}
	return false; // doesn't choose set hours
}
//=====================================================================================================================
void wait_for_releasseTS(void){
	while(XPT2046_TouchPressed() == true); // wait for releasse TS
	HAL_Delay(20);
}
//=====================================================================================================================
bool clear_ss(void){

	if(Pressed_Point.x < 70) {
		if(Pressed_Point.y > 125) {
			if(Pressed_Point.y < 185) {
				read_RTC();
				my_time.Seconds = 0;
				HAL_RTC_SetTime(&hrtc, &my_time, RTC_FORMAT_BIN);
				print_RTC();
				wait_for_releasseTS(); // wait for releasse TS
				return true;
			}
		}
	}
	return false;
}
//=====================================================================================================================
bool set_hours(void) {

	uint8_t action, bckp, chk_hrs;
	read_RTC();
	chk_hrs = my_time.Hours;
	if((test_pressed_point(0, 150, 0, 115)) == true)  {
		TIM4_IRQHandler(); // disable timer4 and IRQ
		LCD_ClrScr(lcd_background_color);
		while(1){
			lcd_text_boxed(55, 5, "SET HOURS", dum1_ttf, 42);
			put_number( my_time.Hours);
			action = test_action();
			if((action == 3)) { // DONE pressed
				bckp = my_time.Hours;
				read_RTC();
				my_time.Hours = bckp;
				if(chk_hrs != my_time.Hours) {
					HAL_RTC_SetTime(&hrtc, &my_time, RTC_FORMAT_BIN); // store setup value
				}
				LCD_ClrScr(lcd_background_color);
				print_RTC();
				return true;
			} else {
				if(action == 2) { // PLUS pressed
					if(my_time.Hours == 23) { my_time.Hours = 0;} else { my_time.Hours++;}
				} else { // MINUS pressed
					if(my_time.Hours == 0) { my_time.Hours = 23;} else { my_time.Hours--;}
				}
			}

		}

	}
	return false; // doesn't choose set hours
}
//=====================================================================================================================
void put_number(uint8_t data){

	lcd_mono_text_boxed(220, 50, "+", digital_7_ttf, 86);
	lcd_mono_text_boxed(0, 50, "-", digital_7_ttf, 86);
	sprintf(tekst, "%02d", data);
	lcd_mono_text_boxed(80, 50, tekst, digital_7_ttf, 86);
	lcd_text_boxed(115, 180, "DONE", dum1_ttf, 42);
	LCD_No_Fill_Draw(lcd_text_color, 80, 172, 240, 230);
	LCD_No_Fill_Draw(lcd_text_color, 0, 55, 80, 165);
	LCD_No_Fill_Draw(lcd_text_color, 235, 55, 318, 165);
}
//=====================================================================================================================
void put_DATE_number(void){

	lcd_mono_text_boxed(220, 50, "+", digital_7_ttf, 86);
	lcd_mono_text_boxed(0, 50, "-", digital_7_ttf, 86);

	lcd_text_boxed(115, 180, "DONE", dum1_ttf, 42);
	LCD_No_Fill_Draw(lcd_text_color, 80, 172, 240, 230);
	LCD_No_Fill_Draw(lcd_text_color, 0, 55, 80, 165);
	LCD_No_Fill_Draw(lcd_text_color, 235, 55, 318, 165);
}
//=====================================================================================================================
uint8_t test_action(void) {
	while(1) {
		get_pressed_point();
		// DONE test
		if((test_pressed_point(80, 240, 172, 230)) == true) { return 3;}
		if((test_pressed_point(235, 318, 55, 165)) == true) { return 2;}
		if((test_pressed_point(0, 80, 55, 165)) == true) { return 1;}
	}
}
//=====================================================================================================================
bool test_pressed_point(uint16_t xmin, uint16_t xmax, uint16_t ymin, uint16_t ymax){
	if(Pressed_Point.x > xmin) {
		if(Pressed_Point.x < xmax){
			if(Pressed_Point.y > ymin){
				if(Pressed_Point.y < ymax){
					return true;
				}
			}
		}
	}
	return false;
}
//=====================================================================================================================



//=====================================================================================================================
//=====================================================================================================================
//=====================================================================================================================
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
