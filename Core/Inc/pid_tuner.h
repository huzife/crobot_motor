#ifndef CROBOT_MOTOR_PID_TUNER_H
#define CROBOT_MOTOR_PID_TUNER_H

#include <stdint.h>
#include <stdbool.h>

#define PID_TUNER_TUNING_CYCLES 20

extern bool pid_tuner_in_progress;
uint16_t pid_tuner_reg_get();
void pid_tuner_reg_set(uint16_t val);
void pid_tuner_start(uint16_t target_value);
bool pid_tuner_loop(int16_t cur_value);

#endif // CROBOT_MOTOR_PID_TUNER_H
