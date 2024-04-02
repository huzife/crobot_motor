/**
 * Motor PWM control and encoder setup
 *
 * Motor is driven using A4950T PWM motor driver, and an
 * encoder is used for each motor to report its position.
 *
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#include <motor_control.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_bus.h>

static void motor_tim_encoder_setup(TIM_TypeDef *TIMx) {
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    LL_TIM_SetEncoderMode(TIMx, LL_TIM_ENCODERMODE_X4_TI12);
    LL_TIM_IC_SetActiveInput(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetActiveInput(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 65535;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIMx, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIMx);
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIMx);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
    LL_TIM_EnableCounter(TIMx);
}

static void motor_tim_encoder_setup_1() {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    motor_tim_encoder_setup(TIM_ENCODER_1);
}

static void motor_tim_encoder_setup_2() {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    motor_tim_encoder_setup(TIM_ENCODER_2);
}

static void motor_tim_encoder_setup_3() {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    motor_tim_encoder_setup(TIM_ENCODER_3);
}

static void motor_tim_encoder_setup_4() {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    motor_tim_encoder_setup(TIM_ENCODER_4);
}

static void motor_tim_pwm_setup(TIM_TypeDef* TIMx) {
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 1000;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIMx, &TIM_InitStruct);
    LL_TIM_EnableARRPreload(TIMx);
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_Init(TIMx, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIMx, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_Init(TIMx, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIMx, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_Init(TIMx, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIMx, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);
    LL_TIM_OC_Init(TIMx, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIMx, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIMx);
    TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
    TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
    TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime = 0;
    TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init(TIMx, &TIM_BDTRInitStruct);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableCounter(TIMx);
    LL_TIM_EnableAllOutputs(TIMx);
}

static void motor_tim_pwm_setup_1() {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10|LL_GPIO_PIN_11;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    motor_tim_pwm_setup(TIM1);
}

static void motor_tim_pwm_setup_2() {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_3;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    motor_tim_pwm_setup(TIM8);
}

/**
 * Setup timers for encoders and PWM output
 */
void motor_timer_setup(void) {
    motor_tim_encoder_setup_1();
    motor_tim_encoder_setup_2();
    motor_tim_encoder_setup_3();
    motor_tim_encoder_setup_4();
    motor_tim_pwm_setup_1();
    motor_tim_pwm_setup_2();
}

/**
 * Set PWM speed for motor 1
 *
 * @param pwm
 */
void motor1_set_pwm(int pwm) {
    if (pwm > 0) {
        LL_TIM_OC_SetCompareCH1(TIM_PWM_1, TIM_PWM_MAXVAL);
        LL_TIM_OC_SetCompareCH2(TIM_PWM_1, TIM_PWM_MAXVAL - pwm);
    } else {
        LL_TIM_OC_SetCompareCH1(TIM_PWM_1, TIM_PWM_MAXVAL + pwm);
        LL_TIM_OC_SetCompareCH2(TIM_PWM_1, TIM_PWM_MAXVAL);
    }
}

/**
 * Set PWM speed for motor 2
 *
 * @param pwm
 */
void motor2_set_pwm(int pwm) {
    if (pwm > 0) {
        LL_TIM_OC_SetCompareCH3(TIM_PWM_2, TIM_PWM_MAXVAL);
        LL_TIM_OC_SetCompareCH4(TIM_PWM_2, TIM_PWM_MAXVAL - pwm);
    } else {
        LL_TIM_OC_SetCompareCH3(TIM_PWM_2, TIM_PWM_MAXVAL + pwm);
        LL_TIM_OC_SetCompareCH4(TIM_PWM_2, TIM_PWM_MAXVAL);
    }
}

/**
 * Set PWM speed for motor 3
 *
 * @param pwm
 */
void motor3_set_pwm(int pwm) {
    if (pwm > 0) {
        LL_TIM_OC_SetCompareCH1(TIM_PWM_3, TIM_PWM_MAXVAL);
        LL_TIM_OC_SetCompareCH2(TIM_PWM_3, TIM_PWM_MAXVAL - pwm);
    } else {
        LL_TIM_OC_SetCompareCH1(TIM_PWM_3, TIM_PWM_MAXVAL + pwm);
        LL_TIM_OC_SetCompareCH2(TIM_PWM_3, TIM_PWM_MAXVAL);
    }
}

/**
 * Set PWM speed for motor 4
 *
 * @param pwm
 */
void motor4_set_pwm(int pwm) {
    if (pwm > 0) {
        LL_TIM_OC_SetCompareCH3(TIM_PWM_4, TIM_PWM_MAXVAL);
        LL_TIM_OC_SetCompareCH4(TIM_PWM_4, TIM_PWM_MAXVAL - pwm);
    } else {
        LL_TIM_OC_SetCompareCH3(TIM_PWM_4, TIM_PWM_MAXVAL + pwm);
        LL_TIM_OC_SetCompareCH4(TIM_PWM_4, TIM_PWM_MAXVAL);
    }
}

/**
 * Put motor 1 into idle mode
 */
void motor1_idle(void) {
    LL_TIM_OC_SetCompareCH1(TIM_PWM_1, 0);
    LL_TIM_OC_SetCompareCH2(TIM_PWM_1, 0);
}

/**
 * Put motor 2 into idle mode
 */
void motor2_idle(void) {
    LL_TIM_OC_SetCompareCH3(TIM_PWM_2, 0);
    LL_TIM_OC_SetCompareCH4(TIM_PWM_2, 0);
}

/**
 * Put motor 3 into idle mode
 */
void motor3_idle(void) {
    LL_TIM_OC_SetCompareCH1(TIM_PWM_3, 0);
    LL_TIM_OC_SetCompareCH2(TIM_PWM_3, 0);
}

/**
 * Put motor 4 into idle mode
 */
void motor4_idle(void) {
    LL_TIM_OC_SetCompareCH3(TIM_PWM_4, 0);
    LL_TIM_OC_SetCompareCH4(TIM_PWM_4, 0);
}
