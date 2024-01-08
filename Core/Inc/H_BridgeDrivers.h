//#include "CollectiveDefines.h"
#include "main.h"

/*					STEROWNIK1 PWM1					*/
//#define DRIVER_1_C_CW_SET		HAL_GPIO_WritePin(DIR_R1_GPIO_Port, DIR_R1_Pin, SET); // dir_r_s1
//#define DRIVER_1_C_CW_RESET		HAL_GPIO_WritePin(DIR_R1_GPIO_Port, DIR_R1_Pin, RESET);
//
//#define DRIVER_1_CW_SET			HAL_GPIO_WritePin(DIR_L1_GPIO_Port, DIR_L1_Pin, SET); // dir_l_s1
//#define DRIVER_1_CW_RESET		HAL_GPIO_WritePin(DIR_L1_GPIO_Port, DIR_L1_Pin, RESET);
//
///*					STEROWNIK2 PWM2`				*/
//#define DRIVER_2_C_CW_SET		HAL_GPIO_WritePin(DIR_R2_GPIO_Port, DIR_R2_Pin, SET);
//#define DRIVER_2_C_CW_RESET		HAL_GPIO_WritePin(DIR_R2_GPIO_Port, DIR_R2_Pin, RESET);
//
//#define DRIVER_2_CW_SET			HAL_GPIO_WritePin(DIR_L2_GPIO_Port, DIR_L2_Pin, SET);
//#define DRIVER_2_CW_RESET		HAL_GPIO_WritePin(DIR_L2_GPIO_Port, DIR_L2_Pin, RESET);
//
///*					STEROWNIK3 PWM3`				*/
//#define DRIVER_3_C_CW_SET		HAL_GPIO_WritePin(DIR_R3_GPIO_Port, DIR_R3_Pin, SET);
//#define DRIVER_3_C_CW_RESET		HAL_GPIO_WritePin(DIR_R3_GPIO_Port, DIR_R3_Pin, RESET);
//
//#define DRIVER_3_CW_SET			HAL_GPIO_WritePin(DIR_L3_GPIO_Port, DIR_L3_Pin, SET);
//#define DRIVER_3_CW_RESET		HAL_GPIO_WritePin(DIR_L3_GPIO_Port, DIR_L3_Pin, RESET);
//
//void HBridge_Direction_PinInit(void);
typedef enum{
	WHEEL_1 = 1,
	WHEEL_2,
	WHEEL_3
}wheel_number;

void wheelGoAhead(wheel_number driver_number);
void wheelGoBack(wheel_number driver_number);
