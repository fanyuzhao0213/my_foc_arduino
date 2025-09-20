#include "as5600.h"

//----------------- 内部变量 -----------------
static const uint8_t _ams5600_Address = 0x36; // I2C 地址
static const uint8_t _raw_ang_hi = 0x0C;
static const uint8_t _raw_ang_lo = 0x0D;

static int32_t full_rotations = 0; // 总旋转圈数
static float angle_prev = 0;       // 上一次角度值

//----------------- 内部函数 -----------------

/**
 * @brief 从 AS5600 读取两个字节数据
 * @param in_adr_hi 高字节地址
 * @param in_adr_lo 低字节地址
 * @return 合并后的16位值
 */
static uint16_t readTwoBytes(uint8_t in_adr_hi, uint8_t in_adr_lo) {
    uint16_t retVal = 0;

    // 读取低字节
    Wire.beginTransmission(_ams5600_Address);
    Wire.write(in_adr_lo);
    Wire.endTransmission();
    Wire.requestFrom(_ams5600_Address, (uint8_t)1);
    while(Wire.available() == 0);
    uint8_t low = Wire.read();

    // 读取高字节
    Wire.beginTransmission(_ams5600_Address);
    Wire.write(in_adr_hi);
    Wire.endTransmission();
    Wire.requestFrom(_ams5600_Address, (uint8_t)1);
    while(Wire.available() == 0);
    uint8_t high = Wire.read();

    retVal = ((uint16_t)high << 8) | low;
    return retVal;
}

//----------------- 对外接口 -----------------
bool AS5600_begin(uint8_t sda, uint8_t scl, uint32_t freq) {
    Wire.begin(sda, scl, freq);
    delay(100);

    Wire.beginTransmission(0x36); // AS5600 默认 I2C 地址
    if (Wire.endTransmission() != 0) {
        Serial.println("AS5600 not detected!");
        return false;
    }
    return true;
}

/**
 * @brief 获取原始 12 位角度值
 */
uint16_t AS5600_getRawAngle(void) {
    return readTwoBytes(_raw_ang_hi, _raw_ang_lo);
}

/**
 * @brief 获取单圈角度（弧度制），不考虑旋转圈数
 */
float AS5600_getAngleSingle(void) {
    uint16_t raw = AS5600_getRawAngle();
    return ((float)raw * 0.08789f * PI / 180.0f); // 转弧度
}

/**
 * @brief 获取总角度（弧度制），累加旋转圈数
 */
float AS5600_getAngle(void) {
    float val = AS5600_getAngleSingle();
    float d_angle = val - angle_prev;

    // 判断是否发生跨越整圈
    if (fabs(d_angle) > (0.8f * 2.0f * PI)) {
        full_rotations += (d_angle > 0) ? -1 : 1;
    }

    angle_prev = val;
    return (float)full_rotations * 2.0f * PI + angle_prev;
}
