/**
 * PID closed loop controller
 *
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#ifndef CROBOT_MOTOR_PID_H
#define CROBOT_MOTOR_PID_H
#include <stdint.h>

#define PID_MAX_INSTANCES 4
#define PID_REG_SCALE 1000.0
/* PID loop should be executed every pid_interval ms. */
extern uint32_t pid_interval;

void pid_set_kp(uint16_t val);
void pid_set_ki(uint16_t val);
void pid_set_kd(uint16_t val);
uint16_t pid_get_kp();
uint16_t pid_get_ki();
uint16_t pid_get_kd();
void pid_reset();

float pid_update(const unsigned int instance, float cur_error);

#endif //CROBOT_MOTOR_PID_H
