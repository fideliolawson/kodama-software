/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "srv_midi_internal.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI4_15_IRQn
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define Photo_resistor1_Pin GPIO_PIN_0
#define Photo_resistor1_GPIO_Port GPIOC
#define Photo_resistor2_Pin GPIO_PIN_1
#define Photo_resistor2_GPIO_Port GPIOC
#define Photo_resistor3_Pin GPIO_PIN_2
#define Photo_resistor3_GPIO_Port GPIOC
#define Photo_resistor4_Pin GPIO_PIN_3
#define Photo_resistor4_GPIO_Port GPIOC
#define Photo_resistor5_Pin GPIO_PIN_0
#define Photo_resistor5_GPIO_Port GPIOA
#define DownPad2_Pin GPIO_PIN_1
#define DownPad2_GPIO_Port GPIOA
#define DownPad2_EXTI_IRQn EXTI0_1_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define UpPad1_Pin GPIO_PIN_6
#define UpPad1_GPIO_Port GPIOA
#define UpPad1_EXTI_IRQn EXTI4_15_IRQn
#define DownPad1_Pin GPIO_PIN_7
#define DownPad1_GPIO_Port GPIOA
#define DownPad1_EXTI_IRQn EXTI4_15_IRQn
#define UpPad2_Pin GPIO_PIN_0
#define UpPad2_GPIO_Port GPIOB
#define UpPad2_EXTI_IRQn EXTI0_1_IRQn
#define Photo_resistor4B10_Pin GPIO_PIN_10
#define Photo_resistor4B10_GPIO_Port GPIOB
#define Photo_resistor4B10_EXTI_IRQn EXTI4_15_IRQn
#define Piezo3_Pin GPIO_PIN_15
#define Piezo3_GPIO_Port GPIOB
#define Piezo3_EXTI_IRQn EXTI4_15_IRQn
#define Piezo2_Pin GPIO_PIN_9
#define Piezo2_GPIO_Port GPIOC
#define Piezo2_EXTI_IRQn EXTI4_15_IRQn
#define Piezo1_Pin GPIO_PIN_8
#define Piezo1_GPIO_Port GPIOA
#define Piezo1_EXTI_IRQn EXTI4_15_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define DownPad4_Pin GPIO_PIN_3
#define DownPad4_GPIO_Port GPIOB
#define UpPad4_Pin GPIO_PIN_5
#define UpPad4_GPIO_Port GPIOB
#define UpPad4_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */
#define PLAY1_NOTE 24
#define PLAY2_NOTE 25
#define PLAY3_NOTE 26
#define PLAY4_NOTE 27
#define PLAY5_NOTE 28
#define LUM1THRESHOLD 1960
#define LUM2THRESHOLD 3600
#define LUM3THRESHOLD 3600
#define LUM4THRESHOLD 3600
#define LUM5THRESHOLD 3600
#define DELAYUPDATEPHOTO 10
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
