#include <pid.h>
#include <pid_tuner.h>
#include <motor_control.h>
#include <math.h>

#define ZN_NO_OVERSHOOT
#if defined(ZN_BASIC_PID)
#define KP_CONST 0.6
#define KI_CONST 0.5
#define KD_CONST 0.125
#elif defined(ZN_LESS_OVERSHOOT)
#define KP_CONST 0.33
#define KI_CONST 0.5
#define KD_CONST 0.33
#elif defined(ZN_NO_OVERSHOOT)
#define KP_CONST 0.2
#define TI_CONST 0.5
#define TD_CONST 0.33
#else
#define KP_CONST 0.45
#define TI_CONST 0.80
#define TD_CONST 0
#endif

bool pid_tuner_in_progress = false;
static bool output_enabled = false;
static uint32_t tuner_loop_count;
static uint32_t tuner_target_value;
static uint32_t t1, t2, t_high, t_low, tick;
static int16_t min_input = 0x7fff;
static int16_t max_input = -0x8000;
static float p_avg, i_avg, d_avg;

void pid_tuner_reg_set(uint16_t val) {
    if (val)
        pid_tuner_start(val);
}

uint16_t pid_tuner_reg_get() {
    if (pid_tuner_in_progress)
        return tuner_target_value;
    return 0;
}

void pid_tuner_start(uint16_t target_value) {
    tuner_target_value = target_value;
    tuner_loop_count = 0;
    output_enabled = true;
    t1 = t2 = 0;
    t_high = t_low = 0;
    tick = 0;
    p_avg = i_avg = d_avg = 0;
    pid_tuner_in_progress = true;
}

bool pid_tuner_loop(int16_t cur_value) {
    float ku, tu, kp, ki, kd;
    tick++;
    if (cur_value < min_input)
        min_input = cur_value;
    if (cur_value > max_input)
        max_input = cur_value;

    if (output_enabled && cur_value > tuner_target_value) {
        output_enabled = false;
        t1 = tick;
        t_high = t1 - t2;
        max_input = tuner_target_value;
    }

    if (!output_enabled && cur_value < tuner_target_value) {
        output_enabled = true;
        t2 = tick;
        t_low = t2 - t1;
        ku = (4.0 * ((TIM_PWM_MAXVAL) / 2.0)) / (M_PI * (max_input - min_input) / 2.0);
        tu = (t_low + t_high) * (pid_interval / 1000.0);

        kp = KP_CONST * ku;
        ki = kp / (TI_CONST * tu);
        kd = TD_CONST * kp * tu;

        if(tuner_loop_count > 1) {
            p_avg += kp;
            i_avg += ki;
            d_avg += kd;
        }
        min_input = tuner_target_value;
        tuner_loop_count++;
    }

    if(tuner_loop_count >= PID_TUNER_TUNING_CYCLES) {
        output_enabled = false;
        pid_set_kp(p_avg / (tuner_loop_count - 1) * PID_REG_SCALE);
        pid_set_ki(i_avg / (tuner_loop_count - 1) * PID_REG_SCALE);
        pid_set_kd(d_avg / (tuner_loop_count - 1) * PID_REG_SCALE);
        pid_tuner_in_progress = false;
    }
    return output_enabled;
}