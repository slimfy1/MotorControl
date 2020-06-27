#ifndef MOTORCONTROL_H
#define MOTORCONTROL

#include "stm32h7xx.h"

void motorInit(void);

void motor1_move(uint8_t dir);
void motor1_enable(void);
void motor1_disable(void);

void motor2_move(uint8_t dir);
void motor2_enable(void);
void motor2_disable(void);

void motor3_move(uint8_t dir);
void motor3_enable(void);
void motor3_disable(void);

void motor4_move(uint8_t dir);
void motor4_enable(void);
void motor4_disable(void);

#endif
