/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t aTxBuffer[3];
uint8_t CC1Value = 60;
uint8_t nbTest = 0;
uint8_t CC2Value = 60;
uint8_t CC3Value = 60;
uint8_t CC4Value = 60;
uint8_t VideoSelector = 0;
uint32_t value[5];
bool UpPad1_state = false;
bool UpPad2_state = false;
bool UpPad3_state = false;
bool UpPad4_state = false;
bool DownPad1_state = false;
bool DownPad2_state = false;
bool DownPad3_state = false;
bool DownPad4_state = false;
bool Piezo1_state = false;
bool Piezo2_state = false;
bool Piezo3_state = false;
bool Note3_state = false;
bool Play1 = false;
bool Play2 = false;
bool Play3 = false;
bool Play4 = false;
bool Play5 = false;
bool initPassed = false;
int lum1average = 0;
int lum2average = 0;
int lum3average = 0;
int lum4average = 0;
int lum5average = 0;
int Lum1threshold = 0;
int Lum2threshold = 0;
int Lum3threshold = 0;
int Lum4threshold = 0;
int Lum5threshold = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI4_15_IRQHandler_Config(void);
void GET_ADC_Value(void);
void ADC_Test(void);
void ADC_Select_CH11 (void);
void ADC_Select_CH10 (void);
void ADC_Select_CH4 (void);
void ADC_Select_CH12 (void);
void ADC_Select_CH13 (void);
void CallibrationPhoto(ADC_HandleTypeDef* hadc);
void PhotoProcess(ADC_HandleTypeDef* hadc);

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  HAL_ADCEx_Calibration_Start(&hadc);
  HAL_TIM_Base_Start_IT(&htim3);
  //HAL_ADC_Start_DMA(&hadc, value, 3);
  //srv_iqs5xx_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_Delay(1000);
//	  srv_midi_internal_sendNote(0x3C, 0x00, 0x5A,huart1);
//	  HAL_Delay(200);
//	  srv_midi_internal_sendNote(0x3C, 0x00, 0x00,huart1);


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */
//
//  /** Configure for the selected ADC regular channel to be converted.
//    */
//    sConfig.Channel = ADC_CHANNEL_10;
//    sConfig.Rank = 2;
//    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//    {
//      Error_Handler();
//    }
//	/** Configure for the selected ADC regular channel to be converted.
//	  */
//  sConfig.Channel = ADC_CHANNEL_4;
//  sConfig.Rank = 1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//	Error_Handler();
//  }
//    /** Configure for the selected ADC regular channel to be converted.
//    */
//    sConfig.Channel = ADC_CHANNEL_11;
//    sConfig.Rank = 2;
//    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//    {
//      Error_Handler();
//    }
//    /** Configure for the selected ADC regular channel to be converted.
//    */
//    sConfig.Channel = ADC_CHANNEL_4;
//    sConfig.Rank = 3;
//    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//    {
//      Error_Handler();
//    }
//    /** Configure for the selected ADC regular channel to be converted.
//        */
//	sConfig.Channel = ADC_CHANNEL_0;
//	sConfig.Rank = 4;
//	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//	{
//	  Error_Handler();
//	}
    /** Configure for the selected ADC regular channel to be converted.
    */
//    sConfig.Channel = ADC_CHANNEL_13;
//    sConfig.Rank = 4;
//    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//    {
//      Error_Handler();
//    }

  /* USER CODE END ADC_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.Init.BaudRate = 31250;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DownPad1_Pin Piezo2_Pin */
  GPIO_InitStruct.Pin = DownPad1_Pin|Piezo2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UpPad1_Pin Piezo1_Pin */
  GPIO_InitStruct.Pin = UpPad1_Pin|Piezo1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : UpPad2_Pin DownPad3_Pin DownPad2_Pin TapMap_Pin
                           Piezo3_Pin */
  GPIO_InitStruct.Pin = UpPad2_Pin|DownPad3_Pin|DownPad2_Pin|TapMap_Pin
                          |Piezo3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : UpPad4_Pin */
  GPIO_InitStruct.Pin = UpPad4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UpPad4_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) {

		case UpPad1_Pin:
			if (HAL_GPIO_ReadPin(UpPad1_GPIO_Port, UpPad1_Pin)== GPIO_PIN_SET && UpPad1_state != true){
				if (CC1Value<126){
					CC1Value +=3;
				}
				srv_midi_internal_controlChange(4, CC1Value, huart1);
				UpPad1_state = true;
				}
			else if (HAL_GPIO_ReadPin(UpPad1_GPIO_Port, UpPad1_Pin)== GPIO_PIN_RESET && UpPad1_state != false) {
				UpPad1_state = false;
			}
			break;

		case UpPad2_Pin:
					if (HAL_GPIO_ReadPin(UpPad2_GPIO_Port, UpPad2_Pin)== GPIO_PIN_SET && UpPad2_state != true){
						CC2Value +=3;
						srv_midi_internal_controlChange(5, CC2Value, huart1);
						UpPad2_state = true;
						}
					else if (HAL_GPIO_ReadPin(UpPad2_GPIO_Port, UpPad2_Pin)== GPIO_PIN_RESET && UpPad2_state != false) {
						UpPad2_state = false;
					}
					break;

//		case UpPad3_Pin:
//							if (HAL_GPIO_ReadPin(UpPad3_GPIO_Port, UpPad3_Pin)== GPIO_PIN_SET && UpPad3_state != true){
//								CC3Value ++;
//								srv_midi_internal_controlChange(6, CC3Value, huart1);
//								UpPad3_state = true;
//								}
//							else if (HAL_GPIO_ReadPin(UpPad3_GPIO_Port, UpPad3_Pin)== GPIO_PIN_RESET && UpPad3_state != false) {
//								UpPad3_state = false;
//							}
//							break;

		case TapMap_Pin:
			if (HAL_GPIO_ReadPin(TapMap_GPIO_Port, TapMap_Pin)== GPIO_PIN_SET && UpPad4_state != true){
				srv_midi_internal_sendNote(VideoSelector, 3, 60, huart1);
				UpPad4_state = true;
				}
			else if (HAL_GPIO_ReadPin(TapMap_GPIO_Port, TapMap_Pin)== GPIO_PIN_RESET && UpPad4_state != false) {
				UpPad4_state = false;

				if (VideoSelector<3){
					srv_midi_internal_sendNote(VideoSelector, 3, 0, huart1);
					VideoSelector++;
				}
				else {
					srv_midi_internal_sendNote(VideoSelector, 3, 0, huart1);
					VideoSelector=0;
				}
			}
			break;

		case DownPad1_Pin:
			if (HAL_GPIO_ReadPin(DownPad1_GPIO_Port, DownPad1_Pin)== GPIO_PIN_SET && DownPad1_state != true){
				if (CC1Value>0){
					CC1Value -=3;
				}
				srv_midi_internal_controlChange(4, CC1Value, huart1);
				DownPad1_state = true;
				}
			else if (HAL_GPIO_ReadPin(DownPad1_GPIO_Port, DownPad1_Pin)== GPIO_PIN_RESET && DownPad1_state != false) {
				DownPad1_state = false;
			}
			break;
//
		case DownPad2_Pin:
					if (HAL_GPIO_ReadPin(DownPad2_GPIO_Port, DownPad2_Pin)== GPIO_PIN_SET && DownPad2_state != true){
						CC2Value -=3;
						srv_midi_internal_controlChange(5, CC2Value, huart1);
						DownPad2_state = true;
						}
					else if (HAL_GPIO_ReadPin(DownPad2_GPIO_Port, DownPad2_Pin)== GPIO_PIN_RESET && DownPad2_state != false) {
						DownPad2_state = false;
					}
					break;

//		case DownPad3_Pin:
//							if (HAL_GPIO_ReadPin(DownPad3_GPIO_Port, DownPad3_Pin)== GPIO_PIN_SET && DownPad3_state != true){
//								CC3Value --;
//								srv_midi_internal_controlChange(6, CC3Value, huart1);
//								DownPad3_state = true;
//								}
//							else if (HAL_GPIO_ReadPin(DownPad3_GPIO_Port, DownPad3_Pin)== GPIO_PIN_RESET && DownPad3_state != false) {
//								DownPad3_state = false;
//							}
//							break;

//		case DownPad4_Pin:
//									if (HAL_GPIO_ReadPin(DownPad4_GPIO_Port, DownPad4_Pin)== GPIO_PIN_SET && DownPad4_state != true){
//										if (VideoSelector>19){
//											VideoSelector--;
//											srv_midi_internal_sendNote(VideoSelector, 3, 60, huart1);
//										}
//										DownPad3_state = true;
//										}
//									else if (HAL_GPIO_ReadPin(DownPad3_GPIO_Port, DownPad3_Pin)== GPIO_PIN_RESET && DownPad3_state != false) {
//										DownPad3_state = false;
//										srv_midi_internal_sendNote(VideoSelector, 3, 0, huart1);
//									}
//									break;

		case Piezo1_Pin:
											if (HAL_GPIO_ReadPin(Piezo1_GPIO_Port, Piezo1_Pin)== GPIO_PIN_RESET && Piezo1_state != true){
												srv_midi_internal_sendNote(60, 2, 60, huart1);
												Piezo1_state = true;
												}
											else if (HAL_GPIO_ReadPin(Piezo1_GPIO_Port, Piezo1_Pin)== GPIO_PIN_SET && Piezo1_state != false) {
												Piezo1_state = false;
												srv_midi_internal_sendNote(60, 2, 0, huart1);
											}
											break;
		case Piezo2_Pin:
											if (HAL_GPIO_ReadPin(Piezo2_GPIO_Port, Piezo2_Pin)== GPIO_PIN_RESET && Piezo2_state != true){
												srv_midi_internal_sendNote(63, 2, 60, huart1);
												Piezo2_state = true;
												}
											else if (HAL_GPIO_ReadPin(Piezo2_GPIO_Port, Piezo2_Pin)== GPIO_PIN_SET && Piezo2_state != false) {
												Piezo2_state = false;
												srv_midi_internal_sendNote(63, 2, 0, huart1);
											}
											break;

		case Piezo3_Pin:
											if (HAL_GPIO_ReadPin(Piezo3_GPIO_Port, Piezo3_Pin)== GPIO_PIN_RESET && Piezo3_state != true){
												srv_midi_internal_sendNote(67, 2, 60, huart1);
												Piezo3_state = true;
												}
											else if (HAL_GPIO_ReadPin(Piezo3_GPIO_Port, Piezo3_Pin)== GPIO_PIN_SET && Piezo3_state != false) {
												Piezo3_state = false;
												srv_midi_internal_sendNote(67, 2, 0, huart1);
											}
											break;

//		case RDY_PIN_Pin: if (HAL_GPIO_ReadPin(RDY_PIN_GPIO_Port, RDY_PIN_Pin)== GPIO_PIN_RESET){
//			srv_iqs5xx_callback();
//		}
		default:
			break;
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
 //Non blocking delay for getting ADC value every x ms
  //ADC_Select_CH10();
  //GET_ADC_Value();
  if (initPassed == false){
	  ADC_Test();
	  CallibrationPhoto(&hadc);

	}
	else {
		ADC_Test();
		PhotoProcess(&hadc);
	}
  HAL_TIM_Base_Stop_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim3);


}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) //Callback when ADC got a value
{

    // Read & Update The ADC Result
	//HAL_Delay(1000);
		if (initPassed == false){
			//CallibrationPhoto(hadc);
			//ADC_Test();
		}
		else {
			//PhotoProcess(hadc);
		}

}

void ADC_Test(void){
	for (int i = 0; i<3; i++){
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
		value[0]=value[0]+HAL_ADC_GetValue(&hadc);
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
		value[1]=value[1]+HAL_ADC_GetValue(&hadc);
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
		value[2]=value[2]+HAL_ADC_GetValue(&hadc);
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
		value[3]=value[3]+HAL_ADC_GetValue(&hadc);
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
		value[4]=value[4]+HAL_ADC_GetValue(&hadc);
	}

	value[0]=value[0]/3;
	value[1]=value[1]/3;
	value[2]=value[2]/3;
	value[3]=value[3]/3;
	value[4]=value[4]/3;
//	HAL_ADC_Start(&hadc);
//	HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
//	value[4]=HAL_ADC_GetValue(&hadc);

//	value[0]=0;
//	value[1]=0;
//	value[2]=0;
//	uint32_t one = 0;
//	uint32_t two = 0;
//	uint32_t three = 0;
	//HAL_ADC_Start_DMA(&hadc, value, 3);
//	one = value[0];s
//	two = value[1];
//	three = value[2];
}

void CallibrationPhoto(ADC_HandleTypeDef* hadc){
	if(nbTest<4){

		if (nbTest !=0){lum1average = value[0] + lum1average;}
		//ADC_Select_CH11();
		//HAL_Delay(DELAYUPDATEPHOTO);
		if (nbTest != 0){lum2average = value[1] + lum2average;}
		//HAL_ADC_Stop(hadc);
		if (nbTest != 0){lum3average = value[2] + lum3average;}

		if (nbTest != 0){lum4average = value[3] + lum4average;}
//
		if (nbTest != 0){lum5average = value[4] + lum5average;}



		nbTest++;
	}
	else {
		Lum1threshold = lum1average/10 - 100;
		Lum2threshold = lum2average/10 - 100;
		Lum3threshold = lum3average/10 - 100;
		Lum4threshold = lum4average/10 - 100;
		Lum5threshold = lum5average/10 - 100;
		initPassed = true;
	}
}

void PhotoProcess(ADC_HandleTypeDef* hadc){

	if (value[0] < Lum1threshold && Play1 !=true){
		srv_midi_internal_sendNote(PLAY1_NOTE, 7, 50, huart1);
		Play1 = true;
	}
	else if (value[0] >= Lum1threshold && Play1 == true) {
		Play1 = false;
		srv_midi_internal_sendNote(PLAY1_NOTE, 7, 0, huart1);
	}

	if (value[1] < Lum2threshold && Play2 !=true){
		srv_midi_internal_sendNote(PLAY2_NOTE, 7, 50, huart1);
		Play2 = true;
	}
	else if (value[1] >= Lum2threshold && Play2 == true) {
		Play2 = false;
		srv_midi_internal_sendNote(PLAY2_NOTE, 7, 0, huart1);
	}


	if (value[2] < Lum3threshold && Play3 !=true){
		srv_midi_internal_sendNote(PLAY3_NOTE, 7, 50, huart1);
		Play3 = true;
	}
	else if (value[2] >= Lum3threshold && Play3 == true) {
		Play3 = false;
		srv_midi_internal_sendNote(PLAY3_NOTE, 7, 0, huart1);
	}

	if (value[3] < Lum4threshold && Play4 !=true){
		srv_midi_internal_sendNote(PLAY4_NOTE, 7, 50, huart1);
		Play4 = true;
	}
	else if (value[3] >= Lum4threshold && Play4 == true) {
		Play4 = false;
		srv_midi_internal_sendNote(PLAY4_NOTE, 7, 0, huart1);
	}

	if (value[4] < Lum5threshold && Play5 !=true){
		srv_midi_internal_sendNote(PLAY5_NOTE, 7, 50, huart1);
		Play5 = true;
	}
	else if (value[4] >= Lum5threshold && Play5 == true) {
		Play5 = false;
		srv_midi_internal_sendNote(PLAY5_NOTE, 7, 0, huart1);
	}

			//ADC_Select_CH13();
			//HAL_ADC_Start(hadc);
//			HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
//			uint16_t LumValue_4 = HAL_ADC_GetValue(hadc);
//			if (LumValue_4 < Lum3threshold && Play4 !=true){
//				srv_midi_internal_sendNote(PLAY4_NOTE, 7, 50, huart1);
//				Play4 = true;
//			}
//			else if (LumValue_4 >= Lum3threshold && Play4 == true) {
//				Play4 = false;
//				srv_midi_internal_sendNote(PLAY4_NOTE, 7, 0, huart1);
//			}
//			HAL_ADC_Stop(hadc);
//
//			ADC_Select_CH0();
//			HAL_ADC_Start(hadc);
//			HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
//			uint16_t LumValue_5 = HAL_ADC_GetValue(hadc);
//			if (LumValue_5 < Lum3threshold && Play5 !=true){
//				srv_midi_internal_sendNote(PLAY5_NOTE, 7, 50, huart1);
//				Play5 = true;
//			}
//			else if (LumValue_5 >= Lum3threshold && Play5 == true) {
//				Play5 = false;
//				srv_midi_internal_sendNote(PLAY5_NOTE, 7, 0, huart1);
//			}



}

void ADC_Select_CH11 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}

void ADC_Select_CH10 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
    sConfig.Channel = ADC_CHANNEL_10;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
}

void ADC_Select_CH12 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_12;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH13 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_13;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH4 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
}

void GET_ADC_Value(void){ //Function to get all ADC values

	HAL_ADC_Start_IT(&hadc);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
