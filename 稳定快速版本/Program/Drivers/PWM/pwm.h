#ifndef __PWM_H
#define __PWM_H

#include "sys.h"
#include "stm32f4xx.h"

#define PWM1  TIM5->CCR1
#define PWM2  TIM5->CCR2

void TIM5_PWM_Init(u32 arr,u32 psc);
void TIM5_FrictionPwmOutp(int16_t pwm1,int16_t pwm2);

#endif

