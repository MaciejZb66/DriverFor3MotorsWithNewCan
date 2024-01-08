/*
 * Emergency_button.h
 *
 *  Created on: Jun 12, 2022
 *      Author: Dell Inspiron
 */
#include <stdio.h>
#ifndef INC_EMERGENCY_BUTTON_H_
#define INC_EMERGENCY_BUTTON_H_

typedef enum{
	POWER_ON = 0,
	POWER_OFF = 1,
	ERROR_BUT = 2,
}PowerStatus;

uint8_t CheckPowerStatus();


#endif /* INC_EMERGENCY_BUTTON_H_ */
