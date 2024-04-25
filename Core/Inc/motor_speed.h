/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#ifndef CROBOT_MOTOR_MOTOR_SPEED_H
#define CROBOT_MOTOR_MOTOR_SPEED_H
#include <stdint.h>
#define MOTOR_SPEED_TIMEOUT 500
void motor1_set_speed(int16_t speed);
void motor2_set_speed(int16_t speed);
void motor3_set_speed(int16_t speed);
void motor4_set_speed(int16_t speed);
int16_t motor_get_cur_speed(unsigned int motor);
int16_t motor_get_tgt_speed(unsigned int motor);
void motor_set_pid_interval(uint32_t interval);
uint16_t motor_get_pid_interval(void);
void motor_set_count_per_rev(uint16_t value);
uint16_t motor_get_count_per_rev(void);
void motor_speed_init(void);
#endif //CROBOT_MOTOR_MOTOR_SPEED_H
