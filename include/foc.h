#ifndef FOC_H
#define FOC_H

#include <Arduino.h>
#include "AS5600.h"

/* ===================== 宏定义 ===================== */

/* 宏定义使用电压还是电流控制*/
#define USE_VOLTAGE_CONTROL             1


#define FOC_CONSTRAIN(val, low, high) ((val)<(low)?(low):((val)>(high)?(high):(val)))
#define FOC_3PI_2  4.71238898038f    // 3*pi/2
#define FOC_PWM_RESOLUTION 8           // 8位精度
#define FOC_PWM_FREQ 30000             // 30 kHz

/* ===================== 结构体定义 ===================== */
typedef struct {
    float voltage_supply;      // 总电压
    float Ualpha;              // α轴电压
    float Ubeta;               // β轴电压
    float Ua;                  // A相电压
    float Ub;                  // B相电压
    float Uc;                  // C相电压
    float zero_electric_angle; // 电角度归零值
    int PP;                    // 极对数
    int DIR;                   // 电机方向
    uint8_t pwmA_pin;          // PWM A相引脚
    uint8_t pwmB_pin;          // PWM B相引脚
    uint8_t pwmC_pin;          // PWM C相引脚
} FOC_Handle_t;
/* ===================== 外部接口函数 ===================== */


typedef struct {

    float M0_motor_angle;       // 当前机械角度（弧度）
    float M0_motor_velocity;    // 当前速度 rad/s
    float M0_motor_current;     //当前输出的iq力矩
    float M1_motor_angle;       // 当前机械角度（弧度）
    float M1_motor_velocity;    // 当前速度 rad/s
    float M1_motor_current;     //当前输出的iq力矩
} my_foc_motorParmTypdef;
extern my_foc_motorParmTypdef my_foc_motorParm;
/**
 * @brief 初始化 FOC 句柄及 PWM
 * @param handle FOC 句柄
 * @param voltage 电源电压
 */
void FOC_Init(FOC_Handle_t *handle, float voltage);

/**
 * @brief 设置三相 PWM 输出
 */
void FOC_SetPWM(FOC_Handle_t *handle, float Ua, float Ub, float Uc);

/**
 * @brief 电压向量控制函数，Ud/Uq -> 三相 PWM
 */
void FOC_SetTorque(FOC_Handle_t *handle, float Uq, float angle_el);

/**
 * @brief 计算电角度（归一化 0~2PI）
 */
float FOC_ElectricalAngle(FOC_Handle_t *handle);

/**
 * @brief 电机零点校准（归零电角度）
 */
void FOC_AlignSensor(FOC_Handle_t *handle, int PP, int DIR);

/**
 * @brief 获取当前机械角度（弧度）
 */
float FOC_GetMechanicalAngle(void);

/**
 * @brief 串口接收用户目标角度
 */
float FOC_SerialReceiveTarget(void);

/**
 * @brief 计算q轴电流
 */
float FOC_calc_Iq(float current_a, float current_b, float angle_el);

float _normalizeAngle(float angle);

void FOC_M0_Get_Angle_Velocity_Current(void);

/*PID封装电机相关函数*/

void FOC_M0_SET_VEL_PID(float P,float I,float D,float ramp,float limit);//速度PID设定
void FOC_M0_SET_ANGLE_PID(float P,float I,float D,float ramp,float limit);//角度PID设定
void FOC_M0_SET_CURRENT_PID(float P,float I,float D,float ramp); //M0电流环PID设置
float FOC_M0_VEL_PID_UPDATE(float error);//M0速度PID接口
float FOC_M0_ANGLE_PID_UPDATE(float error);//M0角度PID接口
float FOC_M0_CURRENT_PID_UPDATE(float error);

/*角度闭环 + 速度闭环（Position + Velocity Loop） */
void DFOC_M0_Set_Velocity_Angle(float target_angle_rad);
/*速度闭环（Velocity Loop）*/
void DFOC_M0_SetVelocity(float target_velocity_rad_s);
/*角度闭环 / 力位控制（Position / Force-Angle Loop）*/
void DFOC_M0_Set_Force_Angle(float target_angle_rad);
/*开环力矩 / 电压输出（Torque Control）*/
void DFOC_M0_SetTorqueVoltage(float torque);
/*开环力矩 / 电流输出（Torque Control）*/
void DFOC_M0_setTorque(float Target);
#endif
