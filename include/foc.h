#ifndef FOC_H
#define FOC_H

#include <stdint.h>
#include "pid.h"
#include "lowpass_filter.h"

/**
 * @brief 电机控制模式枚举
 */
typedef enum {
    MOTOR_SPEED_LOOP,     ///< 单环速度控制
    MOTOR_ANGLE_LOOP,     ///< 单环位置控制
    MOTOR_DOUBLE_LOOP     ///< 双闭环位置控制（角度环+速度环）
} MotorControlMode_t;

/**
 * @brief 电机FOC控制结构体
 */
typedef struct MotorFOC {
    // ------------------- 硬件相关 -------------------
    int pwm_pinA;          ///< 相A PWM引脚
    int pwm_pinB;          ///< 相B PWM引脚
    int pwm_pinC;          ///< 相C PWM引脚
    int sda_pin;           ///< 编码器sda引脚
    int scl_pin;           ///< 编码器scl引脚

    int pole_pairs;        ///< 电机极对数
    int direction;         ///< 电机旋转方向 1=正向 -1=反向
    float voltage_supply;  ///< 直流母线电压 Vbus

    // ------------------- 电压输出 -------------------
    float Ualpha;          ///< α轴电压
    float Ubeta;           ///< β轴电压
    float Ua;              ///< 相A电压
    float Ub;              ///< 相B电压
    float Uc;              ///< 相C电压

    // ------------------- 传感器零点 -------------------
    float zero_electric_angle; ///< 电角度零点偏移

    // ------------------- 目标量 -------------------
    float target_angle;       ///< 目标角度 (rad)
    float target_velocity;    ///< 目标速度 (rad/s)

    // ------------------- 滤波器和PID -------------------
    PIDController* pid_velocity; ///< 速度环PID
    PIDController* pid_angle;    ///< 角度环PID
    LowPassFilter* filter_velocity; ///< 速度滤波器
    float filtered_velocity;       ///< 滤波后的速度

    // ------------------- 控制模式 -------------------
    MotorControlMode_t mode; ///< 控制模式

    // ------------------- 计算速度的变量 -------------------
    unsigned long now_time;         //当前时间
    unsigned long last_time;        //上一次时间
    float now_Velocity;             //当前速度
    float last_Velocity;            //上一次速度
    float last_angle;               //上一次角度

    // ------------------- 成员函数指针 -------------------
    void (*begin)(struct MotorFOC* motor);           ///< 初始化PWM和传感器
    void (*alignSensor)(struct MotorFOC* motor);    ///< 归零校准
    void (*update)(struct MotorFOC* motor);         ///< 主控制更新函数
} MotorFOC_t;

/**
 * @brief 初始化电机FOC实例
 */
void MotorFOC_Init(MotorFOC_t* motor,
                   int pwmA, int pwmB, int pwmC,
                   int sda, int scl,
                   int polePairs, int dir,
                   PIDController* velPID,
                   PIDController* anglePID,
                   LowPassFilter* velFilter,
                   float voltage);

/**
 * @brief 获取电机电角度
 */
float MotorFOC_GetElectricalAngle(MotorFOC_t* motor);

#endif
