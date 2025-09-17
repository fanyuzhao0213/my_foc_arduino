#include "foc.h"
#include <Arduino.h>
#include <math.h>
#include "AS5600.h" // 角度传感器库
#include "pwm.h"

//----------------- 内部变量 -----------------
static float voltage_power_supply; // 电源电压
static float Ualpha, Ubeta = 0;    // α-β 坐标系电压
static float Ua, Ub, Uc = 0;       // 三相电压
static float zero_electric_angle = 0; // 电机零电角度
static int PP = 1;  // 极对数
static int DIR = 1; // 旋转方向
#define MOTOR_PWM1    32
#define MOTOR_PWM2    33
#define MOTOR_PWM3    25

#define _3PI_2 4.71238898038f // 3π/2
#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//----------------- 内部函数 -----------------

/**
 * @brief 将角度归一化到 [0, 2PI]
 */
float _normalizeAngle(float angle){
    float a = fmod(angle, 2*PI); // 取余
    return a >= 0 ? a : (a + 2*PI);
}

/**
 * @brief 设置三相PWM输出
 * @param Ua 相电压 A
 * @param Ub 相电压 B
 * @param Uc 相电压 C
 */
void setPwm(float Ua, float Ub, float Uc) {
    // 限制占空比在 0~1
    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f);
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f);
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f);

    // 写入 PWM 输出 (8-bit)
    pwm_set_duty(channel1, dc_a*255);
    pwm_set_duty(channel2, dc_b*255);
    pwm_set_duty(channel3, dc_c*255);
}

//----------------- 对外接口 -----------------

/**
 * @brief 初始化 PWM 和传感器
 */
void DFOC_Vbus(float power_supply) {
    voltage_power_supply = power_supply;

    pwm_init(MOTOR_PWM1, MOTOR_PWM2, MOTOR_PWM3, 30000, 8); // 30kHz, 8位分辨率
    Serial.println("完成PWM初始化设置");
    AS5600_begin(19, 18, 400000UL); // 初始化角度传感器
}

/**
 * @brief 设置电机q轴电压，实现转矩控制
 */
void setTorque(float Uq, float angle_el) {
    // 限制Uq在 [-Vbus/2, Vbus/2]
    Uq = _constrain(Uq, -voltage_power_supply/2, voltage_power_supply/2);

    // 电角度归一化
    angle_el = _normalizeAngle(angle_el);

    // d-q -> α-β 逆变换（只使用 q 分量）
    Ualpha = -Uq * sin(angle_el);
    Ubeta  =  Uq * cos(angle_el);

    // α-β -> 三相逆克拉克变换
    Ua = Ualpha + voltage_power_supply/2;
    Ub = (sqrt(3)*Ubeta - Ualpha)/2 + voltage_power_supply/2;
    Uc = (-Ualpha - sqrt(3)*Ubeta)/2 + voltage_power_supply/2;

    // 输出到PWM
    setPwm(Ua, Ub, Uc);
}

/**
 * @brief 对电机进行初始归零校准
 */
void DFOC_alignSensor(int _PP, int _DIR) {
    PP = _PP;
    DIR = _DIR;

    // 给一个固定电压转矩，让电机静止
    setTorque(3, _3PI_2);
    delay(3000);
    // 读取当前电角度作为零点
    zero_electric_angle = _electricalAngle();
    // 关闭输出
    setTorque(0, _3PI_2);
    Serial.print("0电角度: ");
    Serial.println(zero_electric_angle);
}

/**
 * @brief 获取电机当前电角度
 */
float _electricalAngle(void) {
    // 使用 AS5600 模块获取单圈角度
    return _normalizeAngle((float)(DIR * PP) * AS5600_getAngleSingle() - zero_electric_angle);
}

/**
 * @brief 获取电机机械角度
 */
float DFOC_M0_Angle(void) {
    // 使用 AS5600 模块获取总角度（全圈累加）
    return AS5600_getAngle();
}
