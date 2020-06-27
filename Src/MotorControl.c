#include "stm32h743xx.h"
#include "main.h"

//Init all Motors (All "EN" - ON, All "INA, INB" - OFF)
void motorInit()
{
	//Motor 1 setup
	MODIFY_REG(GPIOG->ODR, GPIO_ODR_OD12 | GPIO_ODR_OD9, GPIO_ODR_OD0);
	//Motor 2 setup
	MODIFY_REG(GPIOD->ODR, GPIO_ODR_OD1, GPIO_ODR_OD0);
	CLEAR_BIT(GPIOF->ODR, GPIO_ODR_OD0);
	//Motor 3 setup
	MODIFY_REG(GPIOD->ODR, GPIO_ODR_OD7 | GPIO_ODR_OD6, GPIO_ODR_OD4);
	//Motor 4 setup
	CLEAR_BIT(GPIOD->ODR, GPIO_ODR_OD5);
	CLEAR_BIT(GPIOE->ODR, GPIO_ODR_OD3);
	SET_BIT(GPIOC->ODR, GPIO_ODR_OD3);
}
//Motor 1 funcions
void motor1_move(uint8_t dir)
{
	switch(dir)
	{
	//STOP
	case 2:
		CLEAR_BIT(GPIOG->ODR, GPIO_ODR_OD12);
		CLEAR_BIT(GPIOG->ODR, GPIO_ODR_OD9);
		break;
	//CW
	case 1:
		MODIFY_REG(GPIOG->ODR, GPIO_ODR_OD12, GPIO_ODR_OD9);
		break;
	//CCW
	case 0:
		MODIFY_REG(GPIOG->ODR, GPIO_ODR_OD9, GPIO_ODR_OD12);
		break;
	}
}

void motor1_enable()
{
	SET_BIT(GPIOG->ODR, GPIO_ODR_OD0);
}

void motor1_disable()
{
	CLEAR_BIT(GPIOG->ODR, GPIO_ODR_OD0);
}

//Motor 2 funcions
void motor2_move(uint8_t dir)
{
	switch(dir)
	{
	//STOP
	case 2:
		CLEAR_BIT(GPIOD->ODR, GPIO_ODR_OD1);
		CLEAR_BIT(GPIOF->ODR, GPIO_ODR_OD0);
		break;
	//CW
	case 1:
		SET_BIT(GPIOD->ODR, GPIO_ODR_OD1);
		CLEAR_BIT(GPIOF->ODR, GPIO_ODR_OD0);
		break;
	//CCW
	case 0:
		CLEAR_BIT(GPIOD->ODR, GPIO_ODR_OD1);
		SET_BIT(GPIOF->ODR, GPIO_ODR_OD0);
		break;
	}
}

void motor2_enable()
{
	ENABLE_BIT(GPIOD->ODR, GPIO_ODR_OD0);
}

void motor2_disable()
{
	CLEAR_BIT(GPIOD->ODR, GPIO_ODR_OD0);
}

//Motor 3 funcions
void motor3_move(uint8_t dir)
{
	switch(dir)
	{
	//STOP
	case 2:
		CLEAR_BIT(GPIOD->ODR, GPIO_ODR_OD7);
		CLEAR_BIT(GPIOD->ODR, GPIO_ODR_OD6);
		break;
	//CW
	case 1:
		SET_BIT(GPIOD->ODR, GPIO_ODR_OD7);
		CLEAR_BIT(GPIOD->ODR, GPIO_ODR_OD6);
		break;
	//CCW
	case 0:
		CLEAR_BIT(GPIOD->ODR, GPIO_ODR_OD7);
		SET_BIT(GPIOD->ODR, GPIO_ODR_OD6);
		break;
	}
}

void motor3_enable()
{
	ENABLE_BIT(GPIOD->ODR, GPIO_ODR_OD4);
}

void motor3_disable()
{
	CLEAR_BIT(GPIOD->ODR, GPIO_ODR_OD4);
}

//Motor 4 funcions
void motor4_move(uint8_t dir)
{
	switch(dir)
	{
	//STOP
	case 2:
		CLEAR_BIT(GPIOD->ODR, GPIO_ODR_OD5);
		CLEAR_BIT(GPIOE->ODR, GPIO_ODR_OD3);
		break;
	//CW
	case 1:
		SET_BIT(GPIOD->ODR, GPIO_ODR_OD5);
		CLEAR_BIT(GPIOE->ODR, GPIO_ODR_OD3);
		break;
	//CCW
	case 0:
		CLEAR_BIT(GPIOD->ODR, GPIO_ODR_OD5);
		SET_BIT(GPIOE->ODR, GPIO_ODR_OD3);
		break;
	}
}

void motor4_enable()
{
	ENABLE_BIT(GPIOC->ODR, GPIO_ODR_OD3);
}

void motor4_disable()
{
	CLEAR_BIT(GPIOC->ODR, GPIO_ODR_OD3);
}
