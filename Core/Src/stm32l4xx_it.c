/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "H_BridgeDrivers.h"
#include "PWM_Configuration.h"
#include "EncodersInterface.h"
#include "Emergency_button.h"
#include "Regulator.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#include <stdbool.h>
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern	CAN_TxHeaderTypeDef   TxHeader;
extern	uint32_t              TxMailbox;

extern	uint8_t ReceivedData;
extern	CAN_FilterTypeDef canfil;

extern	CAN_RxHeaderTypeDef   RxHeader;
extern	uint8_t               RxData[8];

extern	volatile uint8_t Status;
extern	volatile uint8_t breakActivition;
extern	volatile uint8_t CAN_state;
extern	volatile uint8_t CAN_MainState;

extern uint8_t CANsendingState;
extern volatile uint8_t messageConfirmation;

extern	volatile uint8_t switchForGovernors;
extern	uint8_t driver_encoderStatus;
extern	volatile uint8_t measurementTrigger[3];

extern	struct PI governorMechanicPart_FIRST;
extern	struct PI *mechanicPI_FIRST;

extern	struct PI governorMechanicPart_SECOND;
extern	struct PI *mechanicPI_SECOND;

extern	struct PI governorMechanicPart_THIRD;
extern	struct PI *mechanicPI_THIRD;

extern volatile bool isConnecting;
extern volatile uint16_t cnt_connecting;

uint8_t requireValueCnt[3] = {0};
uint8_t estop = 0;

int16_t enkoder_int;
uint16_t enkoder_u;
int16_t last_en1_count;
int16_t last_en2_count;
int16_t last_en3_count;

uint32_t arrRegisterSave;
uint32_t cmpRegisterSave;

extern LPTIM_HandleTypeDef hlptim1;

extern uint8_t type_of_set_pwm;
extern uint16_t test;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern LPTIM_HandleTypeDef hlptim2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim16;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 TX interrupt.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC channel1 and channel2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

	#define Encoder_Step 512
	#define Freq 12*60
	#define Halfcount 2<<15
  	  if(((int16_t)(DRIVER_1_ENCODER_CNT)-last_en1_count)>64000){
  		mechanicPI_FIRST->TrueVobr = (((float)(((int16_t)(DRIVER_1_ENCODER_CNT) - Halfcount) + (last_en1_count + Halfcount)))*Freq/Encoder_Step);
  	  }else if (((int16_t)(DRIVER_1_ENCODER_CNT)-last_en1_count)< -64000) {
  		mechanicPI_FIRST->TrueVobr = (((float)((-(int16_t)(DRIVER_1_ENCODER_CNT) + Halfcount) - (last_en1_count - Halfcount)))*Freq/Encoder_Step);
  	  }else{
		mechanicPI_FIRST->TrueVobr = (((float)((int16_t)(DRIVER_1_ENCODER_CNT)-last_en1_count))*Freq/Encoder_Step);
  	  }
	mechanicPI_FIRST->Vobr = (((float)((int16_t)DRIVER_1_ENCODER_CNT))*Magic_Number*2); // przelicza na predkosc obrotowa

	mechanicPI_SECOND->Vobr = (((float)((int16_t)DRIVER_2_ENCODER_CNT))*Magic_Number*2);
	if(((int16_t)(DRIVER_2_ENCODER_CNT)-last_en2_count)>64000){
		mechanicPI_SECOND->TrueVobr = (((float)(((int16_t)(DRIVER_2_ENCODER_CNT)-Halfcount)+(last_en2_count + Halfcount)))*Freq/Encoder_Step);
	  }else if (((int16_t)(DRIVER_2_ENCODER_CNT)-last_en2_count)< -64000) {
		mechanicPI_SECOND->TrueVobr = (((float)((-(int16_t)(DRIVER_2_ENCODER_CNT)+Halfcount)-(last_en2_count - Halfcount)))*Freq/Encoder_Step);
	  }else{
		mechanicPI_SECOND->TrueVobr = (((float)((int16_t)(DRIVER_2_ENCODER_CNT)-last_en2_count))*Freq/Encoder_Step);
	  }

	__HAL_LPTIM_DISABLE(&hlptim1);

	mechanicPI_THIRD->Vobr = (((float)((int16_t)DRIVER_3_ENCODER_CNT))*Magic_Number*2);
	if(((int16_t)(DRIVER_3_ENCODER_CNT)-last_en3_count)>64000){
		mechanicPI_THIRD->TrueVobr = (((float)(((int16_t)(DRIVER_3_ENCODER_CNT)-Halfcount)+(last_en3_count + Halfcount)))*Freq/Encoder_Step);
	  }else if (((int16_t)(DRIVER_3_ENCODER_CNT)-last_en3_count)< -64000) {
		mechanicPI_THIRD->TrueVobr = (((float)((-(int16_t)(DRIVER_3_ENCODER_CNT)+Halfcount)-(last_en3_count - Halfcount)))*Freq/Encoder_Step);
	  }else{
		mechanicPI_THIRD->TrueVobr = (((float)((int16_t)(DRIVER_3_ENCODER_CNT)-last_en3_count))*Freq/Encoder_Step);
	  }
	last_en1_count = DRIVER_1_ENCODER_CNT;
	last_en2_count = DRIVER_2_ENCODER_CNT;
	last_en3_count = DRIVER_3_ENCODER_CNT;

	__HAL_LPTIM_ENABLE(&hlptim1);
	__HAL_LPTIM_START_CONTINUOUS(&hlptim1);

	enkoder_int = DRIVER_2_ENCODER_CNT;
	enkoder_u = DRIVER_2_ENCODER_CNT;
	if(mechanicPI_THIRD->TrueVobr != 0){
		test++;
	}

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

// ZADANA PREDKOSC
//  mechanicPI_FIRST->requireValue = 25;
//  mechanicPI_SECOND->requireValue = 25;
//  mechanicPI_THIRD->requireValue = 25;
/////////////////////////////////////
  if(type_of_set_pwm == TYPE_SET_RPM){
	  if(mechanicPI_FIRST->requireValue == 0 ){
		  requireValueCnt[0]++;
		  if (requireValueCnt[0] > 10) {
			  mechanicPI_FIRST->PWM = 0;
			  mechanicPI_FIRST->PIoutput = 0;
		  	  mechanicPI_FIRST->S_k =0.0;
		  }
	  }else requireValueCnt[0] = 0;

	  if(mechanicPI_SECOND->requireValue == 0 ){
		  requireValueCnt[1]++;
		  if (requireValueCnt[1] > 10) {
			  mechanicPI_SECOND->PWM = 0;
			  mechanicPI_SECOND->PIoutput = 0;
			  mechanicPI_SECOND->S_k =0.0;
		  }
	  }else requireValueCnt[1] = 0;

	  if(mechanicPI_THIRD->requireValue == 0 ){
		  requireValueCnt[2]++;
		  if (requireValueCnt[2] > 10) {
			  mechanicPI_THIRD->PWM = 0;
			  mechanicPI_THIRD->PIoutput = 0;
			  mechanicPI_THIRD->S_k =0.0;
		  }
	  }else requireValueCnt[2] = 0;

	  calcPWM(mechanicPI_FIRST);
	  calcPWM(mechanicPI_SECOND);
	  calcPWM(mechanicPI_THIRD);



	//TODO przetestowac dzialanie
	//mechanicPI_FIRST->requireValue = 0; //zeruj po ustawieniu, jezeli nie beda przychodzic wiadomosci to stanie

		if(mechanicPI_FIRST->requireValue >= 0.0f) {
			TIM15->CCR1 = (uint32_t)(mechanicPI_FIRST->PWM);
			wheelGoAhead(WHEEL_1);
		}else{
			TIM15->CCR1 = (uint32_t)(mechanicPI_FIRST->PWM*(-1.0f));
			wheelGoBack(WHEEL_1);
		}

		if(mechanicPI_SECOND->requireValue >= 0.0f) {
			TIM15->CCR2 = (uint32_t)(mechanicPI_SECOND->PWM);
			wheelGoAhead(WHEEL_2);
		}else{
			TIM15->CCR2 = (uint32_t)(mechanicPI_SECOND->PWM*(-1.0f));
			wheelGoBack(WHEEL_2);
		}

		if(mechanicPI_THIRD->requireValue >= 0.0f) {
			TIM16->CCR1 = (uint32_t)(mechanicPI_THIRD->PWM);
			wheelGoAhead(WHEEL_3);
		}else{
			TIM16->CCR1 = (uint32_t)(mechanicPI_THIRD->PWM*(-1.0f));
			wheelGoBack(WHEEL_3);
		}
  }
  if(type_of_set_pwm == TYPE_SET_PWM){

	  	  if(mechanicPI_FIRST->PWM >= 0.0f) {
		  	  TIM15->CCR1 = (uint32_t)(mechanicPI_FIRST->PWM * 2999);
		  	  wheelGoAhead(WHEEL_1);
	  	  }else{
	  		  TIM15->CCR1 = (uint32_t)(mechanicPI_FIRST->PWM *(-1.0f) * 2999);
	  		  wheelGoBack(WHEEL_1);
	  	  }


	  	  if(mechanicPI_SECOND->PWM >= 0.0f) {
	  		  TIM15->CCR2 = (uint32_t)(mechanicPI_SECOND->PWM * 2999);
	  		  wheelGoAhead(WHEEL_2);
	  	  }else{
	  		  TIM15->CCR2 = (uint32_t)(mechanicPI_SECOND->PWM*(-1.0f) * 2999);
	  		  wheelGoBack(WHEEL_2);
	  	  }

	  	  if(mechanicPI_THIRD->PWM >= 0.0f) {
	  		  TIM16->CCR1 = (uint32_t)(mechanicPI_THIRD->PWM * 2999);
	  		  wheelGoAhead(WHEEL_3);
	  	  }else{
	  		  TIM16->CCR1 = (uint32_t)(mechanicPI_THIRD->PWM*(-1.0f) * 2999);
	  		  wheelGoBack(WHEEL_3);
	  	  }
  	  }
	isConnecting = false;
	cnt_connecting ++;
	if ( cnt_connecting > 60)
	{
		mechanicPI_FIRST->requireValue = 0;
		mechanicPI_SECOND->requireValue = 0;
		mechanicPI_THIRD->requireValue = 0;
	}
  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles LPTIM2 global interrupt.
  */
void LPTIM2_IRQHandler(void)
{
  /* USER CODE BEGIN LPTIM2_IRQn 0 */

  /* USER CODE END LPTIM2_IRQn 0 */
  HAL_LPTIM_IRQHandler(&hlptim2);
  /* USER CODE BEGIN LPTIM2_IRQn 1 */

  /* USER CODE END LPTIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
