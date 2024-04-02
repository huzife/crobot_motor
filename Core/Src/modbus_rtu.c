/**
 * MODBUS RTU protocol implementation
 *
 * Supported functions:
 *  03: Read Holding Registers
 *  04: Read Input Registers
 *  06: Preset Single Register
 * All these queries are 8 bytes long so we can use that as
 * frame boundaries instead of timers.
 *
 * Copyright (c) 2021 Chuanhong Guo <gch981213@gmail.com>
 */

#include <modbus_rtu.h>
#include <motor_speed.h>
#include <stm32f4xx_ll_tim.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_usart.h>
#include <stm32f4xx_ll_dma.h>
#include <stm32f4xx_ll_gpio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <modbus_regs.h>
#include <stdbool.h>

#define TIM_RTU TIM6
#define UART_RTU USART2
#define UART_RTU_BAUD 115200
#define RTU_BUFFER_SIZE 80

/**
 * RTU state machine:
 * 0: waiting for address -> 1 or 5
 * 1: waiting for function -> 2 (set expected data length) or 5
 * 2: waiting for data until byte 6 -> 2 or 3
 * 3: waiting for the leftover data -> 3 or 4
 * 4: ready
 * 5: ignore
 */
#define RTU_STATE_ADDR 0
#define RTU_STATE_FUNCTION 1
#define RTU_STATE_DATA_B6 2
#define RTU_STATE_DATA 3
#define RTU_STATE_READY 4
#define RTU_STATE_IGNORE 5

static volatile uint8_t modbus_rtu_recv_state = 0;
static volatile uint16_t modbus_rtu_recv_data_len;
static volatile uint16_t modbus_rtu_expected_data_len;
static volatile uint8_t modbus_rtu_buf[RTU_BUFFER_SIZE];
static uint8_t modbus_rtu_addr = 0x1;

/* Task handle for notification */
static TaskHandle_t rtu_task_handle = NULL;
/* The end time of master request for T3.5 delay */
static TickType_t modbus_rtu_t35_start = 0;

static void modbus_rtu_timer_reset(void);

/**
 * setup GPIO pin used for DE/RE on RS485 transceiver
 */
// static void rs485_trx_init(void) {
//     LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//     GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
//     GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//     GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
//     GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//     LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
//     LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//     LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
// }

/**
 * set RS485 transceiver direction
 * @param is_tx
 */
// static void rs485_trx_set(bool is_tx) {
//     if (is_tx)
//         LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
//     else
//         LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
// }

/**
 * Setup UART for MODBUS
 */
static void modbus_rtu_uart_init(void) {
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
    NVIC_EnableIRQ(USART2_IRQn);

    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_EVEN;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(UART_RTU, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(UART_RTU);
    LL_USART_EnableIT_RXNE(UART_RTU);
    LL_USART_Enable(UART_RTU);
}

void modbus_rtu_uart_rx_cb(void) {
    uint8_t ch;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    modbus_rtu_timer_reset();
    ch = LL_USART_ReceiveData8(UART_RTU);
    /**
    * first byte is slave address.
    * Check it and if it doesn't match, set the state to IGNORE
    */
    if (modbus_rtu_recv_state == RTU_STATE_ADDR) {
        if (ch && (ch != modbus_rtu_addr)) {
            modbus_rtu_recv_state = RTU_STATE_IGNORE;
        } else {
            modbus_rtu_buf[0] = ch;
            modbus_rtu_recv_state = RTU_STATE_FUNCTION;
        }
    } else if (modbus_rtu_recv_state == RTU_STATE_FUNCTION) {
        modbus_rtu_buf[1] = ch;
        modbus_rtu_recv_data_len = 2;
        if (ch == 0x10) {
            modbus_rtu_recv_state = RTU_STATE_DATA_B6;
        } else {
            modbus_rtu_expected_data_len = 8;
            modbus_rtu_recv_state = RTU_STATE_DATA;
        }
    } else if (modbus_rtu_recv_state == RTU_STATE_DATA_B6) {
        modbus_rtu_buf[modbus_rtu_recv_data_len++] = ch;
        if (modbus_rtu_recv_data_len == 7) {
            modbus_rtu_expected_data_len = 7 + 2 + ch;
            if (modbus_rtu_expected_data_len <= RTU_BUFFER_SIZE)
                modbus_rtu_recv_state = RTU_STATE_DATA;
            else
                modbus_rtu_recv_state = RTU_STATE_IGNORE;
        }
    } else if (modbus_rtu_recv_state == RTU_STATE_DATA) {
        modbus_rtu_buf[modbus_rtu_recv_data_len++] = ch;
        if (modbus_rtu_recv_data_len == modbus_rtu_expected_data_len) {
            /* Tell RTU thread about it. */
            modbus_rtu_t35_start = xTaskGetTickCountFromISR();
            vTaskNotifyGiveFromISR(rtu_task_handle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * Setup DMA for UART TX
 */
static void modbus_rtu_uart_tx_dma_init(void) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    NVIC_SetPriority(DMA1_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_6, LL_DMA_CHANNEL_4);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_6, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_6,
                           (uint32_t) modbus_rtu_buf,
                           LL_USART_DMA_GetRegAddr(UART_RTU),
                           LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_STREAM_6));
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_6);
}

static void modbus_rtu_uart_dma_tx_start(uint16_t txlen) {
    // rs485_trx_set(true);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, txlen);
    LL_USART_EnableDMAReq_TX(UART_RTU);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);
}

void modbus_rtu_uart_tc_cb(void) {
    // rs485_trx_set(false);
    modbus_rtu_recv_state = 0;
    LL_USART_DisableIT_TC(UART_RTU);
}

void modbus_rtu_dma_tc_cb(void) {
    LL_USART_ClearFlag_TC(UART_RTU);
    LL_USART_EnableIT_TC(UART_RTU);
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);
}

/**
 * Setup the idle timer for MODBUS RTU
 *
 * According to MODBUS over serial line specification and implementation guide V1.02,
 * For baud rate higher than 19200Bps, T1.5 should be 0.75ms and T3.5 should be 1.75ms.
 * This timer is for T1.5. T3.5 can be handled using FreeRTOS delays.
 */
static void modbus_rtu_timer_init(void) {
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);
    NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM_InitStruct.Prescaler = 168;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 750 - 1;
    LL_TIM_Init(TIM_RTU, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM_RTU);
    LL_TIM_SetTriggerOutput(TIM_RTU, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM_RTU);
    LL_TIM_ClearFlag_UPDATE(TIM_RTU);
    LL_TIM_EnableIT_UPDATE(TIM_RTU);
}

static void modbus_rtu_timer_reset(void) {
    LL_TIM_SetCounter(TIM_RTU, 0);
    LL_TIM_ClearFlag_UPDATE(TIM_RTU);
    LL_TIM_EnableCounter(TIM_RTU);
}

void modbus_rtu_timer_update_cb(void) {
    LL_TIM_DisableCounter(TIM_RTU);
    if (modbus_rtu_recv_state != RTU_STATE_READY)
        modbus_rtu_recv_state = 0;
}

static void modbus_rtu_setup(void) {
    // rs485_trx_init();
    modbus_rtu_timer_init();
    modbus_rtu_uart_init();
    modbus_rtu_uart_tx_dma_init();
}

/**
 * This method calculates CRC for MODBUS RTU
 *
 * Taken from:
 * https://github.com/alejoseb/Modbus-STM32-HAL-FreeRTOS
 *
 * @return uint16_t calculated CRC value for the message
 * @ingroup Buffer
 * @ingroup u8length
 */
static uint16_t modbus_rtu_crc(const volatile uint8_t *Buffer, uint8_t u8length) {
    unsigned int temp, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < u8length; i++) {
        temp = temp ^ Buffer[i];
        for (unsigned char j = 1; j <= 8; j++) {
            flag = temp & 0x0001;
            temp >>= 1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    temp &= 0xFFFF;
    return temp;
}

static uint16_t modbus_rtu_resp_03(uint16_t start, uint16_t len) {
    uint16_t retlen = 2;
    uint16_t reg, val;

    /* byte count */
    modbus_rtu_buf[retlen++] = len * 2;
    for (reg = start; reg < start + len; reg++) {
        if (retlen >= RTU_BUFFER_SIZE - 2)
            return 0;
        val = modbus_get_holding_reg(reg);
        modbus_rtu_buf[retlen++] = val >> 8;
        modbus_rtu_buf[retlen++] = val & 0xff;
    }
    return retlen;
}

static uint16_t modbus_rtu_resp_04(uint16_t start, uint16_t len) {
    uint16_t retlen = 2;
    uint16_t reg, val;

    /* byte count */
    modbus_rtu_buf[retlen++] = len * 2;
    for (reg = start; reg < start + len; reg++) {
        if (retlen >= RTU_BUFFER_SIZE - 2)
            return 0;
        val = modbus_get_input_reg(reg);
        modbus_rtu_buf[retlen++] = val >> 8;
        modbus_rtu_buf[retlen++] = val & 0xff;
    }
    return retlen;
}

static uint16_t modbus_rtu_resp_06(uint16_t reg, uint16_t val) {
    modbus_set_holding_reg(reg, val);
    return 6;
}

static uint16_t modbus_rtu_resp_10(uint16_t reg, uint16_t cnt) {
    uint16_t reg_data_offs = 7;
    uint16_t cur_reg_offs = 0;
    uint16_t reg_val;
    for (; cur_reg_offs < cnt; cur_reg_offs++) {
        if (reg_data_offs >= RTU_BUFFER_SIZE - 2)
            break;
        reg_val = (((uint16_t) modbus_rtu_buf[reg_data_offs]) << 8) | modbus_rtu_buf[reg_data_offs + 1];
        modbus_set_holding_reg(reg + cur_reg_offs, reg_val);
        reg_data_offs += 2;
    }
    return 6;
}

_Noreturn static void modbus_rtu_task(void *param) {
    uint16_t rtu_crc;
    uint16_t rtu_tx_len;
    uint16_t param1, param2;
    rtu_task_handle = xTaskGetCurrentTaskHandle();
    modbus_rtu_setup();

    while (1) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != 1)
            continue;

        /* check CRC */
        rtu_crc = (((uint16_t) modbus_rtu_buf[modbus_rtu_recv_data_len - 1]) << 8) |
                  modbus_rtu_buf[modbus_rtu_recv_data_len - 2];
        if (rtu_crc != modbus_rtu_crc(modbus_rtu_buf, modbus_rtu_recv_data_len - 2)) {
            modbus_rtu_recv_state = 0;
            continue;
        }

        param1 = (((uint16_t) modbus_rtu_buf[2]) << 8) | modbus_rtu_buf[3];
        param2 = (((uint16_t) modbus_rtu_buf[4]) << 8) | modbus_rtu_buf[5];
        /* check command */
        switch (modbus_rtu_buf[1]) {
            case 3:
                rtu_tx_len = modbus_rtu_resp_03(param1, param2);
                break;
            case 4:
                rtu_tx_len = modbus_rtu_resp_04(param1, param2);
                break;
            case 6:
                rtu_tx_len = modbus_rtu_resp_06(param1, param2);
                break;
            case 0x10:
                rtu_tx_len = modbus_rtu_resp_10(param1, param2);
                break;
            default:
                rtu_tx_len = 0;
                break;
        }

        if (rtu_tx_len) {
            rtu_crc = modbus_rtu_crc(modbus_rtu_buf, rtu_tx_len);
            modbus_rtu_buf[rtu_tx_len++] = rtu_crc & 0xff;
            modbus_rtu_buf[rtu_tx_len++] = rtu_crc >> 8;
            vTaskDelayUntil(&modbus_rtu_t35_start, portTICK_PERIOD_MS * 2);
            modbus_rtu_uart_dma_tx_start(rtu_tx_len);
        } else {
            modbus_rtu_recv_state = 0;
        }
    }
}

void modbus_rtu_task_setup(void) {
    xTaskCreate(modbus_rtu_task, "RTU", 128, NULL, 5, NULL);
}
