
#define DRIVER_1_ENCODER_CNT		TIM1->CNT
#define DRIVER_2_ENCODER_CNT		LPTIM1->CNT
#define DRIVER_3_ENCODER_CNT		TIM2->CNT

enum{
	ENCODER_READY = 0,
	ENCODER_PROCESSING=1,
	ENCODER_IDLE=2
};

