/**
 * MODBUS Registers
 *
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#include <modbus_regs.h>
#include <motor_speed.h>
#include <motor_control.h>
#include <pid.h>
#include <pid_tuner.h>

#define PROTO_REV 3

/**
 * get input registers
 * reg:
 *  0: motor1 encoder position
 *  1: motor1 speed (encoder tick per 50ms)
 *  2: motor2 encoder position
 *  3: motor2 speed
 *  0xFF: protocol revision
 *
 * @param reg
 * @return
 */
uint16_t modbus_get_input_reg(uint16_t reg) {
    switch (reg) {
        case 0:
            return (uint16_t) motor_get_cur_speed(0);
        case 1:
            return (uint16_t) motor_get_cur_speed(1);
        case 2:
            return (uint16_t) motor_get_cur_speed(2);
        case 3:
            return (uint16_t) motor_get_cur_speed(3);
        case 0x10:
            return (uint16_t) motor_get_encoder(TIM_ENCODER_1);
        case 0x11:
            return (uint16_t) motor_get_encoder(TIM_ENCODER_2);
        case 0x12:
            return (uint16_t) motor_get_encoder(TIM_ENCODER_3);
        case 0x13:
            return (uint16_t) motor_get_encoder(TIM_ENCODER_4);
        case 0xFF:
            return PROTO_REV;
        default:
            return 0;
    }
}

/**
 * get holding registers
 * reg:
 *  0: motor1 target speed
 *  1: motor2 target speed
 *
 * @param reg
 * @return
 */
uint16_t modbus_get_holding_reg(uint16_t reg) {
    switch (reg) {
        case 0:
        case 1:
        case 2:
        case 3:
            return (uint16_t) motor_get_tgt_speed(reg);
        case 0x10:
            return pid_get_kp();
        case 0x11:
            return pid_get_ki();
        case 0x12:
            return pid_get_kd();
        case 0x13:
            return motor_get_pid_interval();
        case 0x14:
            return motor_get_count_per_rev();
        case 0x20:
            return pid_tuner_reg_get();
        default:
            return 0;
    }
}

/**
 * set holding registers
 *
 * @param reg
 * @param val
 */
void modbus_set_holding_reg(uint16_t reg, uint16_t val) {
    switch (reg) {
        case 0:
            motor1_set_speed((int16_t) val);
            break;
        case 1:
            motor2_set_speed((int16_t) val);
            break;
        case 2:
            motor3_set_speed((int16_t) val);
            break;
        case 3:
            motor4_set_speed((int16_t) val);
            break;
        case 0x10:
            pid_set_kp(val);
            break;
        case 0x11:
            pid_set_ki(val);
            break;
        case 0x12:
            pid_set_kd(val);
            break;
        case 0x13:
            motor_set_pid_interval(val);
            break;
        case 0x14:
            motor_set_count_per_rev(val);
            break;
        case 0x20:
            pid_tuner_reg_set(val);
            break;
        default:
            break;
    }
}
