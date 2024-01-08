/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/*
 * L - 0x001
 * P - 0x002
 */

#define RIGHT
//#define LEFT
#define NUMBER_OF_PARAMETERS 7
#define PERIPHERAL 0x30
#define TEST_ID 0x20
//#define MUSHROOMSTOPREQUEST  0x024
//#define MUSHROOMSTOPVALUE    0x033
//
//#define ID_BUTTONREQUEST 0x25
//#define ID_BUTTONVALUE 0x34



#ifdef RIGHT
	#define ERROR_ID 0x16
	#define INIT_ID 0x26
	#define STATUS_ID 0x36
	#define CONTROLVEL_ID 0x46
	#define DRIVERWHEELVEL_ID 0x56
	#define DISTANCE_ID 0x66

#endif

#ifdef LEFT
	#define ERROR_ID 0x15
	#define INIT_ID 0x25
	#define STATUS_ID 0x35
	#define CONTROLVEL_ID 0x45
	#define DRIVERWHEELVEL_ID 0x55
	#define DISTANCE_ID 0x65

#endif

#define TYPE_SET_PWM 1
#define TYPE_SET_RPM 0
#define ERROR_NO_INIT 1
#define ERROR_OK 0

#define TP		200	//70.0
#define KP_REG 4.0f
#define KI_REG 3.0f
#define MAX_VALUE_REG 3000.0F
enum {
	STOP = 0,
	WAITING=1,
	ACTIVE=2
};

enum {
	CAN_FIRST_DRIVE = 2,
	CAN_SECOND_DRIVE = 4,
	CAN_THIRD_DRIVE = 6,
};

struct PI {
		volatile float actualEvasion;
		volatile float entireEvasion;
		volatile float PIoutput;
		volatile float requireValue;
		volatile uint16_t MaxCurrent;
		volatile float Kp;
		volatile float Ki;
		volatile float Vobr;
		volatile float TrueVobr;
		volatile uint16_t Current;
		volatile float PWM;
		volatile float S_k;
		volatile float u_k;
		volatile float U_min;
		volatile float U_max;
		volatile float x;
		volatile uint8_t currentMeasurementTrigger;
};

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIR_L1_Pin GPIO_PIN_13
#define DIR_L1_GPIO_Port GPIOC
#define QA_ENC2_Pin GPIO_PIN_0
#define QA_ENC2_GPIO_Port GPIOC
#define E_STOP2_Pin GPIO_PIN_1
#define E_STOP2_GPIO_Port GPIOC
#define QB_ENC2_Pin GPIO_PIN_2
#define QB_ENC2_GPIO_Port GPIOC
#define DIR_R2_Pin GPIO_PIN_3
#define DIR_R2_GPIO_Port GPIOC
#define QA_ENC3_Pin GPIO_PIN_0
#define QA_ENC3_GPIO_Port GPIOA
#define QB_ENC3_Pin GPIO_PIN_1
#define QB_ENC3_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_2
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_3
#define PWM2_GPIO_Port GPIOA
#define E_STOP1_Pin GPIO_PIN_4
#define E_STOP1_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_6
#define PWM3_GPIO_Port GPIOA
#define LED_IN_GREEN_Pin GPIO_PIN_4
#define LED_IN_GREEN_GPIO_Port GPIOC
#define LED_IN_ORANGE_Pin GPIO_PIN_5
#define LED_IN_ORANGE_GPIO_Port GPIOC
#define LED_OUT_GREEN_Pin GPIO_PIN_0
#define LED_OUT_GREEN_GPIO_Port GPIOB
#define CS_ACK1_Pin GPIO_PIN_2
#define CS_ACK1_GPIO_Port GPIOB
#define EN_COM_Pin GPIO_PIN_11
#define EN_COM_GPIO_Port GPIOB
#define CS_ACK2_Pin GPIO_PIN_13
#define CS_ACK2_GPIO_Port GPIOB
#define E_STOP4_Pin GPIO_PIN_14
#define E_STOP4_GPIO_Port GPIOB
#define E_STOP3_Pin GPIO_PIN_15
#define E_STOP3_GPIO_Port GPIOB
#define DIR_L3_Pin GPIO_PIN_6
#define DIR_L3_GPIO_Port GPIOC
#define DIR_R3_Pin GPIO_PIN_7
#define DIR_R3_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_9
#define LED_GPIO_Port GPIOC
#define QA_ENC1_Pin GPIO_PIN_8
#define QA_ENC1_GPIO_Port GPIOA
#define QB_ENC1_Pin GPIO_PIN_9
#define QB_ENC1_GPIO_Port GPIOA
#define LED_OUT_ORANGE_Pin GPIO_PIN_10
#define LED_OUT_ORANGE_GPIO_Port GPIOA
#define GPIO_IN_Pin GPIO_PIN_11
#define GPIO_IN_GPIO_Port GPIOA
#define ButtonPow_Pin GPIO_PIN_12
#define ButtonPow_GPIO_Port GPIOA
#define ButtonLog_Pin GPIO_PIN_15
#define ButtonLog_GPIO_Port GPIOA
#define GPIO_OUT_Pin GPIO_PIN_2
#define GPIO_OUT_GPIO_Port GPIOD
#define DIR_R1_Pin GPIO_PIN_5
#define DIR_R1_GPIO_Port GPIOB
#define DIR_L2_Pin GPIO_PIN_6
#define DIR_L2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
