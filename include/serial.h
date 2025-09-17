#ifndef SERIAL_H
#define SERIAL_H

#include <Arduino.h>

/**
 * @brief 初始化串口
 * @param baudrate 波特率
 */
void serial_init(uint32_t baudrate);

/**
 * @brief 非阻塞串口接收
 * 遇到 '\n' 时解析为浮点数
 * @return true 如果收到完整命令
 */
bool serial_process(void);

/**
 * @brief 获取最新接收到的电机目标值
 * @return 浮点数目标值
 */
float serial_getMotorTarget(void);

#endif
