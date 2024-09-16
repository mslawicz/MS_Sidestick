/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
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
extern osEventFlagsId_t gameCtrlEventsHandle;
extern osTimerId_t IMU_timerHandle;
extern ADC_HandleTypeDef hadc1;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CUSTOM_HID_EPIN_SIZE 64
#define CUSTOM_HID_EPOUT_SIZE 64
#define HAT_RESET_Pin GPIO_PIN_6
#define HAT_RESET_GPIO_Port GPIOE
#define PB_NUCLEO_Pin GPIO_PIN_13
#define PB_NUCLEO_GPIO_Port GPIOC
#define TEST1_Pin GPIO_PIN_2
#define TEST1_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define LED_G_Pin GPIO_PIN_0
#define LED_G_GPIO_Port GPIOB
#define PB_YELLOW_Pin GPIO_PIN_12
#define PB_YELLOW_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_14
#define LED_R_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define TOGGLE_RIGHT_Pin GPIO_PIN_6
#define TOGGLE_RIGHT_GPIO_Port GPIOC
#define PB_WHITE_Pin GPIO_PIN_8
#define PB_WHITE_GPIO_Port GPIOC
#define TOGGTLE_LEFT_Pin GPIO_PIN_9
#define TOGGTLE_LEFT_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define INT1_A_G_Pin GPIO_PIN_1
#define INT1_A_G_GPIO_Port GPIOD
#define INT1_A_G_EXTI_IRQn EXTI1_IRQn
#define HAT_RIGHT_Pin GPIO_PIN_9
#define HAT_RIGHT_GPIO_Port GPIOG
#define HAT_LEFT_Pin GPIO_PIN_10
#define HAT_LEFT_GPIO_Port GPIOG
#define HAT_DOWN_Pin GPIO_PIN_12
#define HAT_DOWN_GPIO_Port GPIOG
#define HAT_UP_Pin GPIO_PIN_13
#define HAT_UP_GPIO_Port GPIOG
#define HAT_MID_Pin GPIO_PIN_15
#define HAT_MID_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_7
#define LED_B_GPIO_Port GPIOB
#define HAT_SET_Pin GPIO_PIN_1
#define HAT_SET_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define IMU_DATA_READY_EVENT  1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
