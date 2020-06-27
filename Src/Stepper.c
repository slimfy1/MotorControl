#include "Stepper.h"

#define STEPPER_PWM GPIO_ODR_OD4
#define STEPPER_DIR GPIO_ODR_OD10
#define STEPPER_EN GPIO_ODR_OD10
uint32_t pulseCount = 100;
uint32_t needDel = 1200;
uint32_t needStop = 300;

void stepperMove(uint32_t stps, bool dir, uint16_t stopDelay)
{
	if (dir)
	{
		SET_BIT(GPIOA->ODR, STEPPER_DIR);
	}
	if (!dir)
	{
		CLEAR_BIT(GPIOA->ODR, STEPPER_DIR);
	}
	
	delay_us(10);
	for (int i = 0; i <= stps; i++)
	{
		SET_BIT(GPIOB->ODR, STEPPER_PWM);
		delay_us(needDel);
		CLEAR_BIT(GPIOB->ODR, STEPPER_PWM);
		delay_us(needDel);
	}
	delay_ms(stopDelay);

}
