/**
 * Motor speed control using PID controller

 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#include <motor_speed.h>
#include <motor_control.h>
#include <pid.h>
#include <pid_tuner.h>
#include <FreeRTOS.h>
#include <timers.h>
#include <math.h>

#define CABS(A) ((A) > 0 ? (A) : -(A))

#define MOTOR_SPEED_SCALE 10000.0f
#define PI 3.1415926f

static uint16_t motor_cpr = 1580;

static struct __motor_speed {
    float pwm;
    int16_t position;
    int16_t current_speed;
    int16_t target_speed;
} motor_info[4];

static xTimerHandle motor_pid_handle, motor_timeout_handle;

/**
 * Set target speed for m

 otor 1
 * @param speed encoder count every pid interval
 */
void motor1_set_speed(int16_t speed) {
    float speed_val = speed / MOTOR_SPEED_SCALE;
    motor_info[0].target_speed = speed_val / (2 * PI) * motor_cpr * (pid_interval / 1000.0f);
    if (fabsf(speed_val) > 1e-6f)
        xTimerReset(motor_timeout_handle, 0);
}

/**
 * Set target speed for motor 2
 * @param speed encoder count every pid interval
 */
void motor2_set_speed(int16_t speed) {
    float speed_val = speed / MOTOR_SPEED_SCALE;
    motor_info[1].target_speed = speed_val / (2 * PI) * motor_cpr * (pid_interval / 1000.0f);
    if (fabsf(speed_val) > 1e-6f)
        xTimerReset(motor_timeout_handle, 0);
}

/**
 * Set target speed for motor 3
 * @param speed encoder count every pid interval
 */
void motor3_set_speed(int16_t speed) {
    float speed_val = speed / MOTOR_SPEED_SCALE;
    motor_info[2].target_speed = speed_val / (2 * PI) * motor_cpr * (pid_interval / 1000.0f);
    if (fabsf(speed_val) > 1e-6f)
        xTimerReset(motor_timeout_handle, 0);
}

/**
 * Set target speed for motor 4
 * @param speed encoder count every pid interval
 */
void motor4_set_speed(int16_t speed) {
    float speed_val = speed / MOTOR_SPEED_SCALE;
    motor_info[3].target_speed = speed_val / (2 * PI) * motor_cpr * (pid_interval / 1000.0f);
    if (fabsf(speed_val) > 1e-6f)
        xTimerReset(motor_timeout_handle, 0);
}

/**
 * Cap the output value between -TIM_PWM_MAXVAL and TIM_PWM_MAXVAL
 * @param pwm_output_f input value
 * @return capped value
 */
static inline float motor_pwm_cap(const float pwm_output_f) {
    if (pwm_output_f > TIM_PWM_MAXVAL)
        return TIM_PWM_MAXVAL;
    else if (pwm_output_f < -TIM_PWM_MAXVAL)
        return -TIM_PWM_MAXVAL;
    else
        return pwm_output_f;
}

/**
 * Obtain current motor speed and position
 */
static void motor_info_update(void) {
    int16_t tmp_position;
    /* Calculate current speed */
    tmp_position = motor_get_encoder(TIM_ENCODER_1);
    motor_info[0].current_speed = tmp_position - motor_info[0].position;
    motor_info[0].position = tmp_position;

    tmp_position = motor_get_encoder(TIM_ENCODER_2);
    motor_info[1].current_speed = tmp_position - motor_info[1].position;
    motor_info[1].position = tmp_position;

    tmp_position = motor_get_encoder(TIM_ENCODER_3);
    motor_info[2].current_speed = tmp_position - motor_info[2].position;
    motor_info[2].position = tmp_position;

    tmp_position = motor_get_encoder(TIM_ENCODER_4);
    motor_info[3].current_speed = tmp_position - motor_info[3].position;
    motor_info[3].position = tmp_position;
}

/**
 * Adjust PWM output according to the PID controller
 */
static void motor_pid_update(void) {
    int pwm_out;
    /* PID control */
    motor_info[0].pwm += pid_update(0, motor_info[0].target_speed - motor_info[0].current_speed);
    motor_info[0].pwm = motor_pwm_cap(motor_info[0].pwm);
    /* avoid applying voltage against current rotary direction */
    if (motor_info[0].current_speed && ((motor_info[0].current_speed > 0) ^ (motor_info[0].pwm > 0)))
        motor_info[0].pwm = 0;
    pwm_out = (int) motor_info[0].pwm;
    if ((motor_info[0].current_speed == 0) && (motor_info[0].target_speed == 0))
        motor1_idle();
    else
        motor1_set_pwm(pwm_out);

    motor_info[1].pwm += pid_update(1, motor_info[1].target_speed - motor_info[1].current_speed);
    motor_info[1].pwm = motor_pwm_cap(motor_info[1].pwm);
    if (motor_info[1].current_speed && ((motor_info[1].current_speed > 0) ^ (motor_info[1].pwm > 0)))
        motor_info[1].pwm = 0;
    pwm_out = (int) motor_info[1].pwm;
    if ((motor_info[1].current_speed == 0) && (motor_info[1].target_speed == 0))
        motor2_idle();
    else
        motor2_set_pwm(pwm_out);

    motor_info[2].pwm += pid_update(2, motor_info[2].target_speed - motor_info[2].current_speed);
    motor_info[2].pwm = motor_pwm_cap(motor_info[2].pwm);
    if (motor_info[2].current_speed && ((motor_info[2].current_speed > 0) ^ (motor_info[2].pwm > 0)))
        motor_info[2].pwm = 0;
    pwm_out = (int) motor_info[2].pwm;
    if ((motor_info[2].current_speed == 0) && (motor_info[2].target_speed == 0))
        motor3_idle();
    else
        motor3_set_pwm(pwm_out);

    motor_info[3].pwm += pid_update(3, motor_info[3].target_speed - motor_info[3].current_speed);
    motor_info[3].pwm = motor_pwm_cap(motor_info[3].pwm);
    if (motor_info[3].current_speed && ((motor_info[3].current_speed > 0) ^ (motor_info[3].pwm > 0)))
        motor_info[3].pwm = 0;
    pwm_out = (int) motor_info[3].pwm;
    if ((motor_info[3].current_speed == 0) && (motor_info[3].target_speed == 0))
        motor4_idle();
    else
        motor4_set_pwm(pwm_out);
}

static void motor_pid_tune(void) {
    if(pid_tuner_loop(motor_info[0].current_speed)) {
        motor1_set_pwm(TIM_PWM_MAXVAL);
        motor2_set_pwm(-TIM_PWM_MAXVAL);
    } else {
        motor1_set_pwm(0);
        motor2_set_pwm(0);
    }
}

/**
 * PID control loop
 * @param timer_handle unused.
 */
static void motor_pid_callback(xTimerHandle timer_handle) {
    motor_info_update();
    motor_pid_update();
    // if(pid_tuner_in_progress)
    //     motor_pid_tune();
    // else
    //     motor_pid_update();
}

static void motor_timeout_callback(xTimerHandle timer_handle) {
    motor_info[0].target_speed = 0;
    motor_info[1].target_speed = 0;
    motor_info[2].target_speed = 0;
    motor_info[3].target_speed = 0;
}

int16_t motor_get_cur_speed(unsigned int motor) {
    if (motor > 3)
        return 0;
    float speed = motor_info[motor].current_speed * (2 * PI) / motor_cpr / (pid_interval / 1000.0f);
    return speed * MOTOR_SPEED_SCALE;
}

int16_t motor_get_tgt_speed(unsigned int motor) {
    if (motor > 3)
        return 0;
    float speed = motor_info[motor].target_speed * (2 * PI) / motor_cpr / (pid_interval / 1000.0f);
    return speed * MOTOR_SPEED_SCALE;
}

uint16_t motor_get_pid_interval(void) {
    return (uint16_t)pid_interval;
}

void motor_set_pid_interval(uint32_t interval) {
    uint16_t ki, kd;
    /* set speed to zero */
    motor_info[0].target_speed = 0;
    motor_info[1].target_speed = 0;
    motor_info[2].target_speed = 0;
    motor_info[3].target_speed = 0;
    /* reset all pid calculations */
    pid_reset();
    /* store ki and kd before adjustment */
    ki = pid_get_ki();
    kd = pid_get_kd();
    /* set interval */
    pid_interval = interval;
    /* restore ki and kd */
    pid_set_ki(ki);
    pid_set_kd(kd);
    /* apply interval */
    xTimerChangePeriod(motor_pid_handle, pdMS_TO_TICKS(interval), pdMS_TO_TICKS(50));
}

void motor_set_count_per_rev(uint16_t value) {
    /* set speed to zero */
    motor_info[0].target_speed = 0;
    motor_info[1].target_speed = 0;
    motor_info[2].target_speed = 0;
    motor_info[3].target_speed = 0;
    /* reset all pid calculations */
    pid_reset();
    motor_cpr = value;
}

uint16_t motor_get_count_per_rev() {
    return motor_cpr;
}

void motor_speed_init(void) {
    motor_pid_handle = xTimerCreate("pid", pdMS_TO_TICKS(pid_interval), pdTRUE, (void *) 0, motor_pid_callback);
    if (motor_pid_handle == NULL)
        configASSERT(pdFAIL);
    if (xTimerStart(motor_pid_handle, portMAX_DELAY) != pdPASS)
        configASSERT(pdFAIL);
    motor_timeout_handle = xTimerCreate("motor_timeout", pdMS_TO_TICKS(MOTOR_SPEED_TIMEOUT), pdFALSE, (void *) 1,
                                        motor_timeout_callback);
    if (motor_timeout_handle == NULL)
        configASSERT(pdFAIL);
    if (xTimerStart(motor_timeout_handle, portMAX_DELAY) != pdPASS)
        configASSERT(pdFAIL);
}
