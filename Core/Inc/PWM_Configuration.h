#include "stm32l433xx.h"

void pwmOutputsInit(void);

#define  TIM1_PreemptionPriority	(uint8_t)0x03
#define  TIM1_SubPriority			(uint8_t)0x00

#define DRIVER_1_PWM1	TIM15->CCR1
#define DRIVER_2_PWM2	TIM15->CCR2
#define DRIVER_3_PWM3	TIM16->CCR1
