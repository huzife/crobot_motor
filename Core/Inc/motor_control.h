/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#ifndef CROBOT_MOTOR_MOTOR_CONTROL_H
#define CROBOT_MOTOR_MOTOR_CONTROL_H

#include "stm32f4xx_ll_tim.h"
#include <stdint.h>

#define TIM_ENCODER_1 TIM2
#define TIM_ENCODER_2 TIM3
#define TIM_ENCODER_3 TIM4
#define TIM_ENCODER_4 TIM5
#define TIM_PWM_1 TIM1
#define TIM_PWM_2 TIM1
#define TIM_PWM_3 TIM8
#define TIM_PWM_4 TIM8
#define TIM_PWM_MAXVAL 1000
#define TIM_PWM_MIN_START 70

void motor_timer_setup(void);
void motor1_set_pwm(int pwm);
void motor2_set_pwm(int pwm);
void motor3_set_pwm(int pwm);
void motor4_set_pwm(int pwm);
void motor1_idle(void);
void motor2_idle(void);
void motor3_idle(void);
void motor4_idle(void);

__STATIC_INLINE int16_t motor_get_encoder(TIM_TypeDef *TIMx)
{
    return (int16_t)LL_TIM_GetCounter(TIMx);
}
#endif //CROBOT_MOTOR_MOTOR_CONTROL_H
