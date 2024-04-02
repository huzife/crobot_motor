//
// Created by gch981213 on 2021/3/3.
//

#ifndef CROBOT_MOTOR_MODBUS_RTU_H
#define CROBOT_MOTOR_MODBUS_RTU_H
void modbus_rtu_task_setup(void);
void modbus_rtu_timer_update_cb(void);
void modbus_rtu_uart_rx_cb(void);
void modbus_rtu_dma_tc_cb(void);
void modbus_rtu_uart_tc_cb(void);
#endif //CROBOT_MOTOR_MODBUS_RTU_H
