
#include "Emergency_button.h"
#include "main.h"

/*
 * Button power PA12
 * Button logic PA15
 */




uint8_t CheckPowerStatus(){
	uint8_t Status = 2;
//	GPIO_PinState state;
//	state = HAL_GPIO_ReadPin(ButtonPow_GPIO_Port, ButtonPow_Pin);

	// Zakomentowane bo błąd w hardware
//	if( HAL_GPIO_ReadPin(ButtonLog_GPIO_Port, ButtonLog_Pin) == GPIO_PIN_SET){
//		// jest zasilanie logiczne, bateria dziala
//	}else

	if ( HAL_GPIO_ReadPin(ButtonPow_GPIO_Port, ButtonPow_Pin) == GPIO_PIN_SET ){
		// przycisk nie wcisniety, jest zasilanie
		// wyslij info - mozna jechac
		Status = POWER_ON;
	}else if ( HAL_GPIO_ReadPin(ButtonPow_GPIO_Port, ButtonPow_Pin) == GPIO_PIN_RESET ){
		// przycisk wcisniety - stoj

		Status = POWER_OFF;
	}else{
		Status = ERROR_BUT;
	}
	return Status;

}
