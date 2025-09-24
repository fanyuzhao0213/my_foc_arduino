/**
 * @file AS5600.c
 * @brief AS5600 角度传感器驱动实现
 */

#include "AS5600.h"

/* ===================== 初始化 ===================== */
void AS5600_Init(AS5600_Handle_t *handle, uint8_t sda, uint8_t scl, uint32_t freq, uint8_t address)
{
    handle->i2c_sda_pin = sda;
    handle->i2c_scl_pin = scl;
    handle->i2c_frequency = freq;
    handle->i2c_address = address;
    handle->full_rotations = 0;
    handle->angle_prev = 0.0f;

    Wire.begin(handle->i2c_sda_pin, handle->i2c_scl_pin, handle->i2c_frequency);
    delay(100);  // 确保设备上电稳定
}

/* ===================== 读取高低字节 ===================== */
uint16_t AS5600_ReadTwoBytes(AS5600_Handle_t *handle, uint8_t reg_hi, uint8_t reg_lo)
{
    uint8_t high = 0, low = 0;

    // 低字节
    Wire.beginTransmission(handle->i2c_address);
    Wire.write(reg_lo);
    Wire.endTransmission();
    Wire.requestFrom(handle->i2c_address, (uint8_t)1);
    while (Wire.available() == 0);
    low = Wire.read();

    // 高字节
    Wire.beginTransmission(handle->i2c_address);
    Wire.write(reg_hi);
    Wire.endTransmission();
    Wire.requestFrom(handle->i2c_address, (uint8_t)1);
    while (Wire.available() == 0);
    high = Wire.read();

    return ((uint16_t)high << 8) | low;
}

/* ===================== 获取原始角度 ===================== */
uint16_t AS5600_GetRawAngle(AS5600_Handle_t *handle)
{
    return AS5600_ReadTwoBytes(handle, AS5600_RAW_ANGLE_HI_REG, AS5600_RAW_ANGLE_LO_REG);
}

/* ===================== 获取角度 (0~2π) ===================== */
float AS5600_GetAngleWithoutTrack(AS5600_Handle_t *handle)
{
    return (float)AS5600_GetRawAngle(handle) * AS5600_RADIAN_PER_BIT;
}

/* ===================== 获取累计角度 ===================== */
float AS5600_GetAngle(AS5600_Handle_t *handle)
{
    float current_angle = AS5600_GetAngleWithoutTrack(handle);
    float delta_angle = current_angle - handle->angle_prev;

    if (fabs(delta_angle) > (0.5f * AS5600_2PI)) {
        handle->full_rotations += (delta_angle > 0.0f) ? -1 : 1;
    }

    handle->angle_prev = current_angle;

    return (float)handle->full_rotations * AS5600_2PI + current_angle;
}

/**
 * @brief 获取电机角速度（弧度/秒）
 *
 * 基于累计旋转圈数和上一次采样角度计算瞬时角速度。
 * @param handle AS5600 句柄
 * @return 角速度 (rad/s)
 */
float AS5600_GetVelocity(AS5600_Handle_t *handle)
{
    // 获取当前时间（微秒）
    uint32_t current_ts = micros();
    // 获取当前累计角度（弧度）
    float current_angle = AS5600_GetAngle(handle);

    // 计算时间间隔（秒）
    float dt = (float)(current_ts - handle->vel_angle_prev_ts) * 1e-6f;
    if (dt <= 0.0f) dt = 1e-3f;  // 避免时间异常导致除零
    if (dt >= 0.05f) dt = 0.05;  // 避免时间异常导致除零

    // 计算角度变化（弧度），考虑跨圈情况
    float d_angle = (float)(handle->full_rotations - handle->vel_full_rotations) * AS5600_2PI
                    + (current_angle - handle->vel_angle_prev);

    // 计算角速度
    float velocity = d_angle / dt;

    // 保存当前状态用于下次速度计算
    handle->vel_full_rotations = handle->full_rotations;
    handle->vel_angle_prev = current_angle;
    handle->vel_angle_prev_ts = current_ts;

    return velocity;
}


/* ===================== 测试函数 ===================== */
void AS5600_Test(AS5600_Handle_t *handle)
{
    Serial.println("===== AS5600 测试 =====");
    Serial.print("原始角度 (0-4095): ");
    Serial.println(AS5600_GetRawAngle(handle));
    Serial.print("角度 (0~2PI, rad): ");
    Serial.println(AS5600_GetAngleWithoutTrack(handle), 4);
    Serial.print("累计旋转角度 (rad): ");
    Serial.println(AS5600_GetAngle(handle), 4);
    Serial.println("=========================");
}
