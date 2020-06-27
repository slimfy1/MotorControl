#ifndef STLIB_H
#define STLIB_H
#include "main.h"
void TIM_Init();

void DWT_Init(void);

static __inline uint32_t delta(uint32_t t0, uint32_t t1);

void delay_us(uint32_t us);

void delay_ms(uint32_t us);

int FRQ_CALCULATOR(int frequency);

void TIM_EN();
void TIM_DISABLE();

void SET_PWM(int pwm);

void SET_FRQ(int frq, int pwm);

#endif
