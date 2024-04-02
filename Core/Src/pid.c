/**
 * PID closed loop controller
 *
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */
#include <pid.h>
#include <string.h>

static float kp = 2, ki = 0.6, kd = 0.1;
uint32_t pid_interval = 50;

/* These should have been zeros. No initialization needed. */
static struct __pid_data {
    float error, last_error;
    float derror, dderror, last_derror;
} pid_data[PID_MAX_INSTANCES];

void pid_set_kp(uint16_t val) {
    kp = val / PID_REG_SCALE;
}

void pid_set_ki(uint16_t val) {
    ki = val / PID_REG_SCALE * (pid_interval / 1000.0);
}

void pid_set_kd(uint16_t val) {
    kd = val / PID_REG_SCALE / (pid_interval / 1000.0);
}

uint16_t pid_get_kp() {
    return kp * PID_REG_SCALE;
}

uint16_t pid_get_ki() {
    return ki * PID_REG_SCALE / (pid_interval / 1000.0);
}

uint16_t pid_get_kd() {
    return kd * PID_REG_SCALE * (pid_interval / 1000.0);
}

void pid_reset() {
    memset(pid_data, 0, sizeof(pid_data));
}

float pid_update(const unsigned int instance, float cur_error) {
    pid_data[instance].error = cur_error;
    pid_data[instance].derror = pid_data[instance].error - pid_data[instance].last_error;
    pid_data[instance].dderror = pid_data[instance].derror - pid_data[instance].last_derror;
    pid_data[instance].last_error = pid_data[instance].error;
    pid_data[instance].last_derror = pid_data[instance].derror;

    return (kp * pid_data[instance].derror) + (ki * pid_data[instance].error) + (kd * pid_data[instance].dderror);
}
