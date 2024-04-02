/**
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#ifndef CROBOT_MOTOR_MODBUS_REGS_H
#define CROBOT_MOTOR_MODBUS_REGS_H
#include <stdint.h>

uint16_t modbus_get_input_reg(uint16_t reg);
uint16_t modbus_get_holding_reg(uint16_t reg);
void modbus_set_holding_reg(uint16_t reg, uint16_t val);
#endif //CROBOT_MOTOR_MODBUS_REGS_H
