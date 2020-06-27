/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M4_INB_Pin GPIO_PIN_3
#define M4_INB_GPIO_Port GPIOE
#define Button_Pin GPIO_PIN_13
#define Button_GPIO_Port GPIOC
#define M2_INB_Pin GPIO_PIN_0
#define M2_INB_GPIO_Port GPIOF
#define INA_Pin GPIO_PIN_6
#define INA_GPIO_Port GPIOF
#define M4_EN_Pin GPIO_PIN_3
#define M4_EN_GPIO_Port GPIOC
#define Stepper2_PWM_Pin GPIO_PIN_0
#define Stepper2_PWM_GPIO_Port GPIOA
#define Stepper6_PWM_Pin GPIO_PIN_1
#define Stepper6_PWM_GPIO_Port GPIOA
#define Stepper4_EN_Pin GPIO_PIN_2
#define Stepper4_EN_GPIO_Port GPIOA
#define Stepper6_DIR_Pin GPIO_PIN_4
#define Stepper6_DIR_GPIO_Port GPIOA
#define Stepper3_EN_Pin GPIO_PIN_5
#define Stepper3_EN_GPIO_Port GPIOC
#define Stepper6_EN_Pin GPIO_PIN_0
#define Stepper6_EN_GPIO_Port GPIOB
#define M1_EN_Pin GPIO_PIN_0
#define M1_EN_GPIO_Port GPIOG
#define Stepper1_PWM_Pin GPIO_PIN_9
#define Stepper1_PWM_GPIO_Port GPIOE
#define Stepper5_EN_Pin GPIO_PIN_10
#define Stepper5_EN_GPIO_Port GPIOE
#define Stepper2_DIR_Pin GPIO_PIN_13
#define Stepper2_DIR_GPIO_Port GPIOE
#define Stepper2_EN_Pin GPIO_PIN_15
#define Stepper2_EN_GPIO_Port GPIOE
#define Stepper1_HALL_1_Pin GPIO_PIN_12
#define Stepper1_HALL_1_GPIO_Port GPIOB
#define Stepper1_HALL_1_EXTI_IRQn EXTI15_10_IRQn
#define Stepper1_HALL_2_Pin GPIO_PIN_13
#define Stepper1_HALL_2_GPIO_Port GPIOB
#define Stepper1_HALL_2_EXTI_IRQn EXTI15_10_IRQn
#define Stepper1_HALL_3_Pin GPIO_PIN_15
#define Stepper1_HALL_3_GPIO_Port GPIOB
#define Stepper1_HALL_3_EXTI_IRQn EXTI15_10_IRQn
#define Stepper5_DIR_Pin GPIO_PIN_11
#define Stepper5_DIR_GPIO_Port GPIOD
#define Stepper5_PWM_Pin GPIO_PIN_12
#define Stepper5_PWM_GPIO_Port GPIOD
#define HALL_4_Pin GPIO_PIN_4
#define HALL_4_GPIO_Port GPIOG
#define HALL_4_EXTI_IRQn EXTI4_IRQn
#define HALL_5_Pin GPIO_PIN_5
#define HALL_5_GPIO_Port GPIOG
#define HALL_5_EXTI_IRQn EXTI9_5_IRQn
#define HALL_6_Pin GPIO_PIN_6
#define HALL_6_GPIO_Port GPIOG
#define HALL_6_EXTI_IRQn EXTI9_5_IRQn
#define HALL_7_Pin GPIO_PIN_7
#define HALL_7_GPIO_Port GPIOG
#define HALL_7_EXTI_IRQn EXTI9_5_IRQn
#define Stepper3_PWM_Pin GPIO_PIN_6
#define Stepper3_PWM_GPIO_Port GPIOC
#define Stepper3_DIR_Pin GPIO_PIN_8
#define Stepper3_DIR_GPIO_Port GPIOC
#define Stepper4_DIR_Pin GPIO_PIN_10
#define Stepper4_DIR_GPIO_Port GPIOA
#define M2_EN_Pin GPIO_PIN_0
#define M2_EN_GPIO_Port GPIOD
#define M2_INA_Pin GPIO_PIN_1
#define M2_INA_GPIO_Port GPIOD
#define M3_EN_Pin GPIO_PIN_4
#define M3_EN_GPIO_Port GPIOD
#define M4_INA_Pin GPIO_PIN_5
#define M4_INA_GPIO_Port GPIOD
#define M3_INB_Pin GPIO_PIN_6
#define M3_INB_GPIO_Port GPIOD
#define M3_INA_Pin GPIO_PIN_7
#define M3_INA_GPIO_Port GPIOD
#define M1_INB_Pin GPIO_PIN_9
#define M1_INB_GPIO_Port GPIOG
#define M1_INA_Pin GPIO_PIN_12
#define M1_INA_GPIO_Port GPIOG
#define INB_Pin GPIO_PIN_7
#define INB_GPIO_Port GPIOB
#define Stepper1_EN_Pin GPIO_PIN_0
#define Stepper1_EN_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
