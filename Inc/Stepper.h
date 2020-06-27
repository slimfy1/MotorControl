#ifndef STEPPER_H
#define STEPPER

#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "STLib.h"

void stepperMove(uint32_t stps, bool dir, uint16_t stopDelay);
#endif
