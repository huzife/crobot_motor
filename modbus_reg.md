# 协议说明
通信使用 modbus_rtu 协议，包含一字节地址、一字节功能码、数据以及两字节校验和，其中数据部分高字节在前，校验和低字节在前。协议寄存器为16位，有时需要对数据进行缩放。缩放x倍表示传输的数据为实际数据的x倍。

协议格式如下，其中 N 为总字节数
|     0      |   1    |  ...  |     N-2      |     N-1      |
| :--------: | :----: | :---: | :----------: | :----------: |
| addr(0x01) | 功能码 | 数据  | 校验和低字节 | 校验和高字节 |

# 寄存器
## 输入寄存器
| 地址  |      寄存器内容       |          说明          |
| :---: | :-------------------: | :--------------------: |
| 0x00  |    电机 1 当前转速    | 单位 rad/s，缩放1000倍 |
| 0x01  |    电机 2 当前转速    | 单位 rad/s，缩放1000倍 |
| 0x02  |    电机 3 当前转速    | 单位 rad/s，缩放1000倍 |
| 0x03  |    电机 4 当前转速    | 单位 rad/s，缩放1000倍 |
| 0x10  | 电机 1 当前编码器计数 |                        |
| 0x11  | 电机 2 当前编码器计数 |                        |
| 0x12  | 电机 3 当前编码器计数 |                        |
| 0x13  | 电机 4 当前编码器计数 |                        |
| 0xFF  |       协议版本        |        当前为3         |

## 保持寄存器
| 地址  |    寄存器内容    |          说明          |
| :---: | :--------------: | :--------------------: |
| 0x00  | 电机 1 目标转速  | 单位 rad/s，缩放1000倍 |
| 0x01  | 电机 2 目标转速  | 单位 rad/s，缩放1000倍 |
| 0x02  | 电机 3 目标转速  | 单位 rad/s，缩放1000倍 |
| 0x03  | 电机 4 目标转速  | 单位 rad/s，缩放1000倍 |
| 0x10  |   PID 参数 kp    |       缩放1000倍       |
| 0x11  |   PID 参数 ki    |       缩放1000倍       |
| 0x12  |   PID 参数 kd    |       缩放1000倍       |
| 0x13  |   PID 执行间隔   |        单位 ms         |
| 0x14  | 编码器一圈总计数 |       缩放0.1倍        |
| 0x15  |   电机是否反转   |                        |

# 通信功能
## 读取保持寄存器
功能码: 0x03

请求数据四个字节，前两个字节为起始寄存器地址，后两个字节为寄存器个数 n

响应数据 2n+1 个字节，第一字节为数据长度 2n，后跟 n 个寄存器数据，其中每个寄存器占两个字节，高字节在前

## 读取输入寄存器
功能码: 0x04

请求数据四个字节，前两个字节为起始寄存器地址，后两个字节为寄存器个数 n

响应数据 2n+1 个字节，第一字节为数据长度 2n，后跟 n 个寄存器数据，其中每个寄存器占两个字节

## 写入单个保持寄存器
功能码: 0x06

请求数据四个字节，前两字节为目标寄存器地址，后两字节为写入值

响应数据与请求数据相同

## 写入多个保持寄存器
功能码: 0x10

请求数据 2n+5 个字节，依次为两字节起始寄存器地址、两字节寄存器个数 n、一字节数据长度 2n、以及 n 个写入值，每个写入值占用两个字节