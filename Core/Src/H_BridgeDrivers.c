#include "H_BridgeDrivers.h"


void wheelGoAhead(wheel_number driver_number){
	switch(driver_number) {
	case WHEEL_1:
		HAL_GPIO_WritePin(DIR_R1_GPIO_Port, DIR_R1_Pin, SET);
		HAL_GPIO_WritePin(DIR_L1_GPIO_Port, DIR_L1_Pin, RESET);
		break;
	case WHEEL_2:
		HAL_GPIO_WritePin(DIR_R2_GPIO_Port, DIR_R2_Pin, SET);
		HAL_GPIO_WritePin(DIR_L2_GPIO_Port, DIR_L2_Pin, RESET);
		break;
	case WHEEL_3:
		HAL_GPIO_WritePin(DIR_R3_GPIO_Port, DIR_R3_Pin, SET);
		HAL_GPIO_WritePin(DIR_L3_GPIO_Port, DIR_L3_Pin, RESET);
		break;
	default:
		break;
	}
}

void wheelGoBack(wheel_number driver_number) {
	switch(driver_number) {
		case WHEEL_1:
			HAL_GPIO_WritePin(DIR_R1_GPIO_Port, DIR_R1_Pin, RESET);
			HAL_GPIO_WritePin(DIR_L1_GPIO_Port, DIR_L1_Pin, SET);
			break;
		case WHEEL_2:
			HAL_GPIO_WritePin(DIR_R2_GPIO_Port, DIR_R2_Pin, RESET);
			HAL_GPIO_WritePin(DIR_L2_GPIO_Port, DIR_L2_Pin, SET);
			break;
		case WHEEL_3:
			HAL_GPIO_WritePin(DIR_R3_GPIO_Port, DIR_R3_Pin, RESET);
			HAL_GPIO_WritePin(DIR_L3_GPIO_Port, DIR_L3_Pin, SET);
			break;
		default:
			break;
		}
}
