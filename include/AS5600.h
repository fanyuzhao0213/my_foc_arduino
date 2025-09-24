/**
 * @file AS5600.h
 * @brief AS5600 角度传感器驱动头文件（C语言结构体封装）
 */

#ifndef __AS5600_H__
#define __AS5600_H__

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

/* ===================== 宏定义 ===================== */
#define AS5600_RAW_ANGLE_HI_REG   0x0C
#define AS5600_RAW_ANGLE_LO_REG   0x0D
#define AS5600_RADIAN_PER_BIT     (0.08789f * PI / 180.0f)   // 0.08789度/bit -> 弧度
#define AS5600_2PI                (2.0f * PI)

/* ===================== 结构体 ===================== */
typedef struct {
    uint8_t i2c_sda_pin;
    uint8_t i2c_scl_pin;
    uint32_t i2c_frequency;
    uint8_t i2c_address;
    int32_t full_rotations;   // 累计旋转圈数
    float angle_prev;         // 上次角度值

    // 以下为速度计算相关变量
    int32_t vel_full_rotations; // 上一次计算速度时的整圈数
    float vel_angle_prev;       // 上一次计算速度时的角度值
    uint32_t vel_angle_prev_ts; // 上一次计算速度的时间戳（微秒）
} AS5600_Handle_t;

/* ===================== 接口函数 ===================== */
void AS5600_Init(AS5600_Handle_t *handle, uint8_t sda, uint8_t scl, uint32_t freq, uint8_t address);
uint16_t AS5600_ReadTwoBytes(AS5600_Handle_t *handle, uint8_t reg_hi, uint8_t reg_lo);
uint16_t AS5600_GetRawAngle(AS5600_Handle_t *handle);
float AS5600_GetAngleWithoutTrack(AS5600_Handle_t *handle);
float AS5600_GetAngle(AS5600_Handle_t *handle);
float AS5600_GetVelocity(AS5600_Handle_t *handle);

/* ===================== 测试函数 ===================== */
void AS5600_Test(AS5600_Handle_t *handle);
#endif // __AS5600_H__
