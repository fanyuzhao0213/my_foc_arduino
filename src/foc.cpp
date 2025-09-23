#include "foc.h"
#include "pwm.h"
#include "AS5600.h"
#include <math.h>
#include <Arduino.h>

// ------------------- 内部工具函数 -------------------

/**
 * @brief 将角度归一化到 [0, 2π)
 */
static float _normalizeAngle(float angle) {
    float a = fmodf(angle, 2.0f * 3.1415926f);
    return (a >= 0) ? a : (a + 2.0f * 3.1415926f);
}

/**
 * @brief 设置三相PWM占空比
 */
static void _setPwm(MotorFOC_t* motor, float Ua, float Ub, float Uc) {
    float dc_a = fmaxf(0.0f, fminf(Ua / motor->voltage_supply, 1.0f));
    float dc_b = fmaxf(0.0f, fminf(Ub / motor->voltage_supply, 1.0f));
    float dc_c = fmaxf(0.0f, fminf(Uc / motor->voltage_supply, 1.0f));

    pwm_set_duty(motor->pwm_pinA, dc_a * 255);
    pwm_set_duty(motor->pwm_pinB, dc_b * 255);
    pwm_set_duty(motor->pwm_pinC, dc_c * 255);
}

/**
 * @brief 根据q轴电压设置三相电压输出
 */
static void _setTorque(MotorFOC_t* motor, float Uq, float angle_el) {
    if (Uq > motor->voltage_supply / 2.0f) Uq = motor->voltage_supply / 2.0f;
    else if (Uq < -motor->voltage_supply / 2.0f) Uq = -motor->voltage_supply / 2.0f;

    angle_el = _normalizeAngle(angle_el);

    motor->Ualpha = -Uq * sinf(angle_el);
    motor->Ubeta  =  Uq * cosf(angle_el);

    motor->Ua = motor->Ualpha + motor->voltage_supply / 2.0f;
    motor->Ub = (sqrtf(3.0f)*motor->Ubeta - motor->Ualpha)/2.0f + motor->voltage_supply / 2.0f;
    motor->Uc = (-motor->Ualpha - sqrtf(3.0f)*motor->Ubeta)/2.0f + motor->voltage_supply / 2.0f;

    _setPwm(motor, motor->Ua, motor->Ub, motor->Uc);
}

/**
 * @brief 获取电角度
 */
static float _getElectricalAngle(MotorFOC_t* motor) {
    return _normalizeAngle(motor->direction * motor->pole_pairs * AS5600_getAngleSingle() - motor->zero_electric_angle);
}

// ------------------- 成员函数实现 -------------------
/**
 * @brief 初始化电机硬件接口
 * @param motor 指向 MotorFOC_t 结构体
 *
 * 初始化 PWM 输出和 AS5600 角度传感器。
 */
static void _motorBegin(MotorFOC_t* motor) {
    // 初始化三相 PWM，频率 30kHz，分辨率 8bit
    pwm_init(motor->pwm_pinA, motor->pwm_pinB, motor->pwm_pinC, 30000, 8);
    // 初始化 AS5600 传感器，SDA=19, SCL=18, I2C速度 400kHz
    AS5600_begin(motor->sda_pin, motor->scl_pin, 400000UL);
    Serial.println("[MotorFOC] PWM和传感器初始化完成");
}

/**
 * @brief 对电机进行归零校准
 * @param motor 指向 MotorFOC_t 结构体
 *
 * 电机校准流程：
 * 1. 施加小电压锁定转子（防止转子自由旋转）
 * 2. 延时 3 秒，让转子稳定
 * 3. 读取当前电角度作为零点
 * 4. 停止输出电压
 */
static void _motorAlign(MotorFOC_t* motor) {
    // 施加 25% Vbus 锁定电机在 3π/2 位置
    _setTorque(motor, motor->voltage_supply * 0.25f, 4.7123889f); // 3π/2
    delay(3000); // 等待转子稳定

    // 记录零电角度
    motor->zero_electric_angle = _getElectricalAngle(motor);

    // 停止输出电压
    _setTorque(motor, 0, 4.7123889f);

    Serial.print("[MotorFOC] 零点校准完成: ");
    Serial.println(motor->zero_electric_angle);
}

#define VEL_MAX  5.0f   // 最大速度 rad/s，根据电机选定
#define VEL_MIN  0.01f   // 最小有效速度 rad/s，低于忽略
/**
 * @brief 电机实时更新函数
 * @param motor 指向 MotorFOC_t 结构体
 *
 * 根据当前控制模式（速度环/角度环/双闭环），计算目标电压 Uq 并输出 PWM。
 */
static void _motorUpdate(MotorFOC_t* motor)
{
    // ---------- 1. 获取当前机械角度 ----------
    float actual_angle = AS5600_getAngle();

    // ---------- 2. 时间增量 ----------
    motor->now_time = micros();
    float dt = (motor->now_time - motor->last_time) * 1e-6f; // 秒
    if(dt <= 0.0f) dt = 1e-6f;  // 防止除以零

    // ---------- 3. 计算瞬时速度 ----------
    float delta_angle = actual_angle - motor->last_angle;

    // 考虑旋转跨越 0/2π
    if(delta_angle > PI) delta_angle -= 2*PI;
    else if(delta_angle < -PI) delta_angle += 2*PI;

    float raw_velocity = delta_angle / dt;

    // ---------- 4. 速度限幅 ----------
    if(fabs(raw_velocity) > VEL_MAX) raw_velocity = VEL_MAX * (raw_velocity > 0 ? 1 : -1);
    else if(fabs(raw_velocity) < VEL_MIN) raw_velocity = 0;

    // ---------- 5. 低通滤波 ----------
    motor->filtered_velocity = LowPassFilter_Update(motor->filter_velocity, raw_velocity);

    // ---------- 6. 更新上一周期数据 ----------
    motor->last_angle = actual_angle;
    motor->last_time = motor->now_time;

    float Uq = 0; // 初始化输出电压

    // ---------- 3. 根据控制模式计算 Uq ----------
    switch(motor->mode) {
        case MOTOR_SPEED_LOOP: {
            // 单环速度控制
            float err = motor->target_velocity - motor->filtered_velocity;      // 速度误差
            Uq = PID_Update(motor->pid_velocity, err);                // 速度 PID 输出
            break;
        }
        case MOTOR_ANGLE_LOOP: {
            // 单环角度控制
            float err = motor->target_angle - actual_angle;           // 角度误差
            if(err > PI)
                err -= 2*PI;
            else if(err < -PI)
                err += 2*PI;
            Uq = PID_Update(motor->pid_angle, err);                  // 角度 PID 输出
            break;
        }
        case MOTOR_DOUBLE_LOOP: {
            // 双闭环位置控制
            float err = motor->target_angle - actual_angle;           // 角度误差
            if(err > PI) err -= 2*PI;
            else if(err < -PI) err += 2*PI;
            // 角度环 PID 输出作为目标速度
            float target_vel = PID_Update(motor->pid_angle, err);

            // 速度环 PID 输出为电压 Uq
            float vel_err = target_vel - motor->filtered_velocity;
            Uq = PID_Update(motor->pid_velocity, vel_err);
            break;
        }
    }

    // ---------- 4. 输出电压到电机 ----------
    _setTorque(motor, Uq, _getElectricalAngle(motor));
}


// ------------------- 公共初始化 -------------------
/**
 * @brief 初始化一个 MotorFOC 电机结构体
 *
 * 该函数用于对 MotorFOC_t 结构体进行初始化，包括：
 * - PWM 引脚
 * - 电机参数（极对数、方向）
 * - PID 控制器指针
 * - 速度滤波器指针
 * - 电源电压
 * - 初始状态
 * - 函数接口指针
 *
 * @param motor 指向 MotorFOC_t 结构体的指针
 * @param pwmA 相A PWM 引脚
 * @param pwmB 相B PWM 引脚
 * @param pwmC 相C PWM 引脚
 * @param polePairs 电机极对数
 * @param dir 电机旋转方向（1=正向，-1=反向）
 * @param velPID 指向速度 PID 控制器的指针
 * @param anglePID 指向角度 PID 控制器的指针
 * @param velFilter 指向速度低通滤波器的指针
 * @param voltage 电机供电电压（Vbus）
 */
void MotorFOC_Init(MotorFOC_t* motor,
                   int pwmA, int pwmB, int pwmC,
                   int sda, int scl,
                   int polePairs, int dir,
                   PIDController* velPID,
                   PIDController* anglePID,
                   LowPassFilter* velFilter,
                   float voltage)
{
    // ---------- PWM 引脚配置 ----------
    motor->pwm_pinA = pwmA;  // 设置相A的PWM引脚
    motor->pwm_pinB = pwmB;  // 设置相B的PWM引脚
    motor->pwm_pinC = pwmC;  // 设置相C的PWM引脚

    // ---------- 编码器IIC 引脚配置 ----------
    motor->sda_pin = sda;
    motor->scl_pin = scl;

    // ---------- 电机基本参数 ----------
    motor->pole_pairs = polePairs;      // 电机极对数
    motor->direction = dir;             // 电机旋转方向
    motor->voltage_supply = voltage;    // 电源电压

    // ---------- PID 与滤波器指针 ----------
    motor->pid_velocity = velPID;       // 速度环 PID 指针
    motor->pid_angle = anglePID;        // 角度环 PID 指针
    motor->filter_velocity = velFilter; // 速度滤波器指针

    // ---------- 初始化状态变量 ----------
    motor->zero_electric_angle = 0.0f;  // 电角度零点
    motor->target_angle = 0.0f;         // 目标角度
    motor->target_velocity = 0.0f;      // 目标速度
    motor->filtered_velocity = 0.0f;    // 滤波后的速度
    motor->mode = MOTOR_DOUBLE_LOOP;    // 默认控制模式：双闭环位置控制

    // ---------- 函数接口绑定 ----------
    motor->begin = _motorBegin;         // 初始化函数
    motor->alignSensor = _motorAlign;   // 电机零点校准函数
    motor->update = _motorUpdate;       // 电机实时更新函数（控制循环）
}


float MotorFOC_GetElectricalAngle(MotorFOC_t* motor) {
    return _getElectricalAngle(motor);
}
