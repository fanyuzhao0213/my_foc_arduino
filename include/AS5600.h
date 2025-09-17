#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <Wire.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 AS5600 角度传感器
 * @param sda SDA 引脚
 * @param scl SCL 引脚
 * @param freq I2C 通信频率，默认 400kHz
 */
void AS5600_begin(uint8_t sda, uint8_t scl, uint32_t freq);

/**
 * @brief 获取原始 12 位角度值
 * @return 0~4095
 */
uint16_t AS5600_getRawAngle(void);

/**
 * @brief 获取单圈弧度制角度（0~2PI），不考虑旋转圈数
 * @return 弧度制角度
 */
float AS5600_getAngleSingle(void);

/**
 * @brief 获取总角度（弧度制），考虑旋转圈数
 * @return 弧度制总角度
 */
float AS5600_getAngle(void);

#ifdef __cplusplus
}
#endif

#endif
