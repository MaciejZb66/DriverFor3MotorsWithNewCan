/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "can.h"
#include "lptim.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "Emergency_button.h"
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
extern	CAN_TxHeaderTypeDef   TxHeader;
extern	uint32_t              TxMailbox;

extern	uint8_t ReceivedData;
extern	CAN_FilterTypeDef canfil;

extern	CAN_RxHeaderTypeDef   RxHeader;
extern	uint8_t RxData[8];

extern uint8_t estop;

volatile uint8_t Status = 0;
volatile uint8_t breakActivition = 0;

uint8_t switchForGovernors = 0;
uint8_t driver_encoderStatus = 0;
volatile uint8_t measurementTrigger[5]={0};

volatile uint8_t CAN_state = 0;
volatile uint8_t CAN_MainState = 0;
uint8_t CANsendingState = 0;
volatile uint8_t messageConfirmation=0;


struct PI governorMechanicPart_FIRST = {0};
struct PI *mechanicPI_FIRST = &governorMechanicPart_FIRST;

struct PI governorMechanicPart_SECOND = {0};
struct PI *mechanicPI_SECOND = &governorMechanicPart_SECOND;

struct PI governorMechanicPart_THIRD = {0};
struct PI *mechanicPI_THIRD = &governorMechanicPart_THIRD;

volatile bool isConnecting = false;
volatile uint16_t cnt_connecting = 0;

volatile uint8_t prev_velocity_cnt = 0;
volatile uint8_t velocity_cnt = 1;

volatile uint8_t tab_id[20];
volatile uint8_t test_id[20];
/*10Hz*/
uint32_t wait;
uint8_t* ptrFloat;
uint8_t  TxData[8];

uint16_t test = 0;
float test_velocity = 0;
float parameter_rpm_scale = 0;
float parameter_rpm_scale_en = 0;
uint8_t type_of_set_pwm = 0;
float parameter_motor_vel_A[NUMBER_OF_PARAMETERS] = {0};
float parameter_motor_vel_B[NUMBER_OF_PARAMETERS] = {0};
float parameter_motor_vel_C[NUMBER_OF_PARAMETERS] = {0};

//uint16_t precision;
uint32_t test_data[6];
uint32_t error_num[51];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

uint8_t Error_check(){
	uint8_t error = ERROR_OK;
	if(parameter_rpm_scale == 0){
		error = ERROR_NO_INIT;
	}
	if(parameter_rpm_scale_en == 0){
		error = ERROR_NO_INIT;
	}
	if(error == ERROR_NO_INIT){
		measurementTrigger[0] = 1;
	}
	return error;
}

struct NewStruct{
	uint8_t RData : 8;
	float temp;
} __attribute__((packed));

union toFloat{
	uint8_t data[8];
	struct NewStruct unitVar;
};
//void TestDataIncrement(void){
//	if(test_data[0] == 0xFFFFFFFF){
//		test_data[0] = 0;
//		test_data[1]++;
//	}else{
//		test_data[0]++;
//	}
//}

//#ifdef RIGHT
//void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim){ //testowe cykliczne wiadomości
//	if(hlptim == &hlptim2){
//		TxHeader.DLC = 8;
//		TxHeader.StdId = TEST_ID;
//		//test_data[0] = 5000;
//		TxData[0] = (uint8_t)test_data[0];
//		TxData[1] = (uint8_t)(test_data[0] >> 8);
//		TxData[2] = (uint8_t)(test_data[0] >> 16);
//		TxData[3] = (uint8_t)(test_data[0] >> 24);
//		TxData[4] = (uint8_t)test_data[1];
//		TxData[5] = (uint8_t)(test_data[1] >> 8);
//		TxData[6] = (uint8_t)(test_data[1] >> 16);
//		TxData[7] = (uint8_t)(test_data[1] >> 24);
//		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox)!= HAL_OK){
//			test_data[3]++;
//		}
//		 __asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
//		 __asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
//		TestDataIncrement();
//	}
//}
//void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
//	test_data[2]++;
//}
//void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
//	test_data[2]++;
//}
//void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
//	test_data[2]++;
//}
//
//void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan){
//	test_data[3]++;
//}
//void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan){
//	test_data[3]++;
//}
//void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan){
//	test_data[3]++;
//}
//#endif

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
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_LPTIM1_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_LPTIM2_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_LPTIM_Encoder_Start(&hlptim1, 65535);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	test_data[0] = 0;
	test_data[1] = 0;
	test_data[2] = 0;
	test_data[3] = 0;
	test_data[4] = 0;
	test_data[5] = 0;
	error_num[0] = 0;


	//lptim f=2,5M/wartość poniżej

//	HAL_LPTIM_Counter_Start_IT(&hlptim2, 1599); //714

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if (CheckPowerStatus() == POWER_OFF)
		{
			mechanicPI_FIRST->requireValue =  0;
			mechanicPI_SECOND->requireValue = 0;
			mechanicPI_THIRD->requireValue = 0;
		}
		if(measurementTrigger[0]==1){
			measurementTrigger[0] = 0;
			TxHeader.StdId = ERROR_ID;
			TxHeader.DLC = 3;
			TxData[0] = 0;
			TxData[1] = 0;
			TxData[2] = 0;
//			uint8_t count_no_parameter = 0;
//			for(int i = 0; i < NUMBER_OF_PARAMETERS; i++){
//				if(parameter_motor_vel_C[i] == 0){
//					count_no_parameter++;
//					TxData[0] |= 0x80;
//				}
//			}
//			if(count_no_parameter == NUMBER_OF_PARAMETERS){
//				RxData[2] |= 0xF0;
//			}

			if(parameter_rpm_scale == 0){
				TxData[1] |= 0x01;
				TxData[0] |= 0x10;
			}
			if(parameter_rpm_scale_en == 0){
				TxData[1] |= 0x02;
				TxData[0] |= 0x10;
			}
			if(parameter_rpm_scale == 0 && parameter_rpm_scale_en == 0){
				TxData[1] |= 0x0F;
			}

			 HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
			 __asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
			 __asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
			TxHeader.DLC = 8;

		}

		if(measurementTrigger[5]==1){//nowy format
			measurementTrigger[5]=0;
			TxHeader.DLC = 7;
			uint16_t tempVobr;
			float max_rpm;
			max_rpm = ((2 << 15) - 1)/ parameter_rpm_scale_en;
			TxHeader.StdId = DRIVERWHEELVEL_ID;
			if(mechanicPI_THIRD->Vobr < max_rpm || mechanicPI_SECOND->Vobr < max_rpm || mechanicPI_FIRST->Vobr < max_rpm){
				tempVobr = (uint16_t)(mechanicPI_THIRD->Vobr * parameter_rpm_scale_en * (mechanicPI_THIRD->Vobr < 0 ? -1: 1));
				TxData[6] = 0x7F & (uint8_t)(tempVobr >> 8);
				TxData[5] = (uint8_t)(tempVobr);
				if(mechanicPI_THIRD->Vobr < 0){//dir
					TxData[6] |= 0x80;
				}
				tempVobr = (uint16_t)(mechanicPI_SECOND->Vobr * parameter_rpm_scale_en * (mechanicPI_SECOND->Vobr < 0 ? -1: 1));
				TxData[4] = 0x7F & (uint8_t)(tempVobr >> 8);
				TxData[3] = (uint8_t)(tempVobr);
				if(mechanicPI_SECOND->Vobr < 0){//dir
					TxData[4] |= 0x80;
				}
				tempVobr = (uint16_t)(mechanicPI_FIRST->Vobr * parameter_rpm_scale_en * (mechanicPI_FIRST->Vobr < 0 ? -1: 1));
				TxData[2] = 0x7F & (uint8_t)(tempVobr >> 8);
				TxData[1] = (uint8_t)(tempVobr);
				if(mechanicPI_FIRST->Vobr < 0){//dir
					TxData[2] |= 0x80;
				}
				TxData[0] = 0;
			}else{
				tempVobr = (uint16_t)(mechanicPI_THIRD->Vobr * parameter_rpm_scale_en / 5 * (mechanicPI_THIRD->Vobr < 0 ? -1: 1));
				TxData[6] = 0x7F & (uint8_t)(tempVobr >> 8);
				TxData[5] = (uint8_t)(tempVobr);
				if(mechanicPI_THIRD->Vobr < 0){//dir
				TxData[6] |= 0x80;
				}
				tempVobr = (uint16_t)(mechanicPI_SECOND->Vobr * parameter_rpm_scale_en / 5 * (mechanicPI_SECOND->Vobr < 0 ? -1: 1));
				TxData[4] = 0x7F & (uint8_t)(tempVobr >> 8);
				TxData[3] = (uint8_t)(tempVobr);
				if(mechanicPI_SECOND->Vobr < 0){//dir
					TxData[4] |= 0x80;
				}
				tempVobr = (uint16_t)(mechanicPI_FIRST->Vobr * parameter_rpm_scale_en / 5 * (mechanicPI_FIRST->Vobr < 0 ? -1: 1));
				TxData[2] = 0x7F & (uint8_t)(tempVobr >> 8);
				TxData[1] = (uint8_t)(tempVobr);
				if(mechanicPI_FIRST->Vobr < 0){//dir
					TxData[2] |= 0x80;
				}
				TxData[0] = 0x01;
			}
			 HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
			 __asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
			 __asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
			 TxHeader.DLC = 8;
		}

//		if(measurementTrigger[0]==1){
//
//			uint8_t  TxData[8] = {};
//			uint8_t* ptrFloat = NULL;
//			measurementTrigger[0]=0;
//
//			ptrFloat =(uint8_t*)&(mechanicPI_FIRST->Vobr);
//
//			TxHeader.StdId = DRIVERWHEEL1VEL;
//			TxData[0] = *(ptrFloat);
//			TxData[1] = *(ptrFloat+1);
//			TxData[2] = *(ptrFloat+2);
//			TxData[3] = *(ptrFloat+3);
//			TxData[4] = 0;
//			TxData[5] = 0;
//			TxData[6] = 0;
//			TxData[7] = 0;
//			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
//			__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
//			__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
//
//
//			 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//		}
//
//		if(measurementTrigger[1]==1){
//
//			uint8_t  TxData[8] = {};
//			uint8_t* ptrFloat = NULL;
//			measurementTrigger[1]=0;
//
//			ptrFloat = (uint8_t*)&(mechanicPI_SECOND->Vobr);
//
//			TxHeader.StdId = DRIVERWHEEL2VEL;
//			TxData[0] = *(ptrFloat);
//			TxData[1] = *(ptrFloat+1);
//			TxData[2] = *(ptrFloat+2);
//			TxData[3] = *(ptrFloat+3);
//			TxData[4] = 0;
//			TxData[5] = 0;
//			TxData[6] = 0;
//			TxData[7] = 0;
//			test_velocity +=1;
//			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
//			__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
//			__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
//
//		}
//
//		if(measurementTrigger[2]==1){
//
//			uint8_t  TxData[8] = {};
//			uint8_t* ptrFloat = NULL;
//			measurementTrigger[2]=0;
//
//			ptrFloat = (uint8_t*)&(mechanicPI_THIRD->Vobr);
//			TxHeader.StdId = DRIVERWHEEL3VEL;
//			TxData[0] = *(ptrFloat);
//			TxData[1] = *(ptrFloat+1);
//			TxData[2] = *(ptrFloat+2);
//			TxData[3] = *(ptrFloat+3);
//			TxData[4] = 0;
//			TxData[5] = 0;
//			TxData[6] = 0;
//			TxData[7] = 0;
//			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
//			__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
//			__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
//
//		}
//
//		if(measurementTrigger[3]==1){
//
//			measurementTrigger[3]=0;
//			uint8_t buttonStatus = CheckPowerStatus();
//
//			TxHeader.StdId = MUSHROOMSTOPVALUE;
//			TxData[0] = buttonStatus;
//			TxData[1] = 0;
//			TxData[2] = 0;
//			TxData[3] = 0;
//			TxData[4] = 0;
//			TxData[5] = 0;
//			TxData[6] = 0;
//			TxData[7] = 0;
//			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
//			__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
//			__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
//
//		}

  /* USER CODE END 3 */
  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

/* USER CODE BEGIN 4 */


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
			static int cnt_test = 0;

		  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		  {
			Error_Handler();
		  }

		  isConnecting = true;
		  cnt_connecting= 0;
//#ifdef LEFT
//		  if((RxHeader.StdId == TEST_ID)){
//			  test_data[4] = test_data[5];
//			  test_data[5] = ((uint32_t)RxData[3] << 24)+((uint32_t)RxData[2] << 16)+((uint32_t)RxData[1] << 8)+((uint32_t)RxData[0]);
//			  if((test_data[5] - test_data[4]) != 1){
//				  error_num[0]++;
//				  if(error_num[0] <= 50){
//					  error_num[error_num[0]] = test_data[5];
//				  }
//			  }
//		  }
//#endif
	  if ((RxHeader.StdId == CONTROLVEL_ID)) {
		  ReceivedData = 1;

	  }

		if (RxHeader.StdId == CONTROLVEL_ID){
			if(Error_check() == ERROR_OK) {
				if((RxData[0] & 0x03) == 0){
					type_of_set_pwm = TYPE_SET_RPM;
					mechanicPI_FIRST->requireValue = (RxData[1] & 0x80? 1 : -1)*((float)((RxData[0] & 0xF0 >> 4)|((uint16_t)(RxData[1] & 0x7F) << 4)))/parameter_rpm_scale; //new
					// znak - 7bit 1bajt dane - 0-6bit 1bajt + 4-7bit 0bajt
					mechanicPI_SECOND->requireValue = (RxData[3] & 0x08? 1 : -1)*((float)((RxData[2])|((uint16_t)(RxData[3] & 0x07) << 8)))/parameter_rpm_scale; //new
					// znak - 3bit 3bajt dane - 0-2bit 3bajt + caly 2bajt
					mechanicPI_THIRD->requireValue = (RxData[4] & 0x80? 1 : -1)*((float)((RxData[3] & 0xF0 >> 4)|((uint16_t)(RxData[4] & 0x7F) << 4)))/parameter_rpm_scale;//new
					// znak - 7bit 4bajt dane - 0-6bit 4bajt + 4-7bit 3bajt
				}
				if((RxData[0] & 0x03) == 3){
					type_of_set_pwm = TYPE_SET_PWM;
					//rzeźba do poprawy na strukurach
					mechanicPI_FIRST->PWM = (float)((uint16_t)((RxData[0] & 0xF0) >> 4)|((uint16_t)(RxData[1] & 0x7F))<< 4)* (RxData[1] & 0x80? -1 : 1) /2047;
					mechanicPI_SECOND->PWM = (float)((RxData[2])|((uint16_t)(RxData[3] & 0x07) << 8))* (RxData[3] & 0x08? -1 : 1) /2047;
					mechanicPI_THIRD->PWM = (float)((uint16_t)((RxData[3] & 0xF0) >> 4)|((uint16_t)(RxData[4] & 0x7F) << 4))*(RxData[4] & 0x80? -1 : 1) /2047;
				}

				velocity_cnt = RxData[7];
				tab_id[cnt_test] = RxData[7];
				test_id[cnt_test] = RxHeader.StdId;
				cnt_test++;
				if (cnt_test == 19) cnt_test = 0;

	#ifdef RIGHT
				mechanicPI_FIRST->requireValue = -mechanicPI_FIRST->requireValue;
				mechanicPI_SECOND->requireValue = -mechanicPI_SECOND->requireValue;
				mechanicPI_THIRD->requireValue = -mechanicPI_THIRD->requireValue;
	#endif

				if( prev_velocity_cnt == velocity_cnt)
				{
					mechanicPI_FIRST->requireValue =  0;
					mechanicPI_SECOND->requireValue = 0;
					mechanicPI_THIRD->requireValue = 0;
				}

				prev_velocity_cnt = velocity_cnt;
				velocity_cnt = 0;

				Status = ACTIVE;
			}
		}
		if(RxHeader.StdId == INIT_ID){
			//float temp = (float)(((uint32_t)(RxData[4])<<24)|((uint32_t)(RxData[3])<<16)|((uint32_t)(RxData[2])<<8)|((uint32_t)(RxData[1])));
		 	union toFloat temp;
		 	for(int i =0 ;i < 8; i++){
		 		temp.data[i] = RxData[i];
		 	}

			if(RxData[0] == 0x10){
					parameter_rpm_scale = temp.unitVar.temp;
				}
				if(RxData[0] == 0x11){
					parameter_rpm_scale_en = temp.unitVar.temp;
				}
//			switch (RxData[0]| 0xF0){//nie działa switch
//			case 0x10:
//
//				break;
//			case 0x20:
//				if((RxData[0] | 0x0F) < NUMBER_OF_PARAMETERS){
//					parameter_motor_vel_A[RxData[0] | 0x0F] = temp;
//				}
//				break;
//			case 0x30:
//				if((RxData[0] | 0x0F) < NUMBER_OF_PARAMETERS){
//				parameter_motor_vel_B[RxData[0] | 0x0F] = temp;
//				}
//				break;
//			case 0x40:
//				if((RxData[0] | 0x0F) < NUMBER_OF_PARAMETERS){
//				parameter_motor_vel_C[RxData[0] | 0x0F] = temp;
//				}
//				break;
//			}
			if(RxData[0] == 0x00){//deinit
				parameter_rpm_scale = 0;
				parameter_rpm_scale_en = 0;
				for(int i = 0; i < NUMBER_OF_PARAMETERS; i++){
					parameter_motor_vel_A[i] = 0;
					parameter_motor_vel_B[i] = 0;
					parameter_motor_vel_C[i] = 0;
				}

			}

		}

		if(RxHeader.StdId == ERROR_ID && RxHeader.RTR == CAN_RTR_REMOTE){
			measurementTrigger[0]=1;
		}
		if (RxHeader.StdId == DRIVERWHEELVEL_ID && RxHeader.RTR == CAN_RTR_REMOTE){
			if(Error_check() == ERROR_OK) {
				measurementTrigger[5]=1;
			}
		}
		if(RxHeader.StdId == DRIVERWHEELVEL_ID && RxHeader.RTR == CAN_RTR_REMOTE){
			if(Error_check() == ERROR_OK) {

			}
		}


		if (RxHeader.StdId == 128){ // 0x80b
			messageConfirmation=1;
		}

	#ifdef LEFT
//		if (RxHeader.StdId == MUSHROOMSTOPREQUEST){
//			measurementTrigger[3]=1;
//		}
//
//		if (RxHeader.StdId == ID_BUTTONREQUEST){
//			measurementTrigger[4]=1;
//		}
	#endif

	//#ifdef RIGHT
	//	if (RxHeader.StdId == MUSHROOMSTOPVALUE){
	//		if(RxData[0]==1)
	//		{
	//			mechanicPI_FIRST->requireValue =  0;
	//			mechanicPI_SECOND->requireValue = 0;
	//			mechanicPI_THIRD->requireValue = 0;
	//		}
	//	}
	//#endif
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
