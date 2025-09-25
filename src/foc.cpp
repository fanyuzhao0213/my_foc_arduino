#include "foc.h"
#include "pwm.h"
#include <math.h>
#include <Arduino.h>
#include "AS5600.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "Current_Sensor.h"


my_foc_motorParmTypdef my_foc_motorParm = {0.0};

/* 定义 AS5600 句柄 */
AS5600_Handle_t M0_as5600_handle;
/* 定义 PID 句柄 */
PIDController M0_speed_pid_controller;
PIDController M0_angle_pid_controller;
PIDController M0_current_pid_controller;
/* 定义 低通滤波 句柄 */
LowPassFilter M0_speed_lowpass_filter;
LowPassFilter M0_Iqcurrent_lowpass_filter;
/* 定义 电流传感器 句柄*/
CurrSense_t M0_current_value;

/* ===================== 全局变量 ===================== */
static FOC_Handle_t g_foc_handle;
static float g_motor_target = 0;

float g_M0_Velocity = 0;
float g_M0_Angle = 0;
float g_M0_Current = 0;
float g_M0_Iq_Current = 0;

/* ===================== PWM 初始化 ===================== */
void FOC_Init(FOC_Handle_t *handle, float voltage)
{
    handle->voltage_supply = voltage;
    /* 默认 PWM 引脚 */
    handle->pwmA_pin = 32;
    handle->pwmB_pin = 33;
    handle->pwmC_pin = 25;
    /* 默认极对数与方向,还有零角度为1，1，0 */
    handle->PP = 1;
    handle->DIR = 1;
    handle->zero_electric_angle = 0;
    /* 设置 PWM */
    pinMode(handle->pwmA_pin, OUTPUT);
    pinMode(handle->pwmB_pin, OUTPUT);
    pinMode(handle->pwmC_pin, OUTPUT);
    ledcSetup(0, FOC_PWM_FREQ, FOC_PWM_RESOLUTION);
    ledcSetup(1, FOC_PWM_FREQ, FOC_PWM_RESOLUTION);
    ledcSetup(2, FOC_PWM_FREQ, FOC_PWM_RESOLUTION);
    ledcAttachPin(handle->pwmA_pin, 0);
    ledcAttachPin(handle->pwmB_pin, 1);
    ledcAttachPin(handle->pwmC_pin, 2);
    Serial.println("FOC PWM 初始化完成");

    // 初始化 AMS5600，SDA=19, SCL=18, I2C 400kHz, 地址 0x36
    AS5600_Init(&M0_as5600_handle, 19, 18, 400000UL, 0x36);

    /* 初始化PID 角度环和速度环*/
    PID_Init(&M0_speed_pid_controller, 2, 0, 0, 100000, voltage/2);
    PID_Init(&M0_angle_pid_controller, 2, 0, 0, 100000, 100);
    PID_Init(&M0_current_pid_controller, 5, 200, 0, 100000, 1);


    /* 初始化低通滤波器 */
    LowPassFilter_Init(&M0_speed_lowpass_filter, 0.05f);
    LowPassFilter_Init(&M0_Iqcurrent_lowpass_filter, 0.05f);

    /* 初始化电流传感器 */
    CurrSense_Init(&M0_current_value, 0);
}

/* ===================== 设置三相 PWM ===================== */
void FOC_SetPWM(FOC_Handle_t *handle, float Ua, float Ub, float Uc)
{
    float dc_a = FOC_CONSTRAIN(Ua / handle->voltage_supply, 0.0f, 1.0f);
    float dc_b = FOC_CONSTRAIN(Ub / handle->voltage_supply, 0.0f, 1.0f);
    float dc_c = FOC_CONSTRAIN(Uc / handle->voltage_supply, 0.0f, 1.0f);

    ledcWrite(0, dc_a * 255);
    ledcWrite(1, dc_b * 255);
    ledcWrite(2, dc_c * 255);
}

/* ===================== 电压向量控制 ===================== */
void FOC_SetTorque(FOC_Handle_t *handle, float Uq, float angle_el)
{
    Uq = FOC_CONSTRAIN(Uq, -handle->voltage_supply / 2, handle->voltage_supply / 2);
    float Ud = 0;

    angle_el = _normalizeAngle(angle_el);

    /* 帕克逆变换 */
    handle->Ualpha = -Uq * sin(angle_el);
    handle->Ubeta  =  Uq * cos(angle_el);

    /* 克拉克逆变换 */
    handle->Ua = handle->Ualpha + handle->voltage_supply / 2;
    handle->Ub = (sqrt(3) * handle->Ubeta - handle->Ualpha) / 2 + handle->voltage_supply / 2;
    handle->Uc = (-handle->Ualpha - sqrt(3) * handle->Ubeta) / 2 + handle->voltage_supply / 2;

    FOC_SetPWM(handle, handle->Ua, handle->Ub, handle->Uc);
}

/* ===================== 电角度计算 ===================== */
float FOC_ElectricalAngle(FOC_Handle_t *handle)
{
    return _normalizeAngle((float)(handle->DIR * handle->PP) * AS5600_GetAngleWithoutTrack(&M0_as5600_handle) - handle->zero_electric_angle);
}

/* ===================== 零点校准 ===================== */
void FOC_AlignSensor(FOC_Handle_t *handle, int PP, int DIR)
{
    handle->PP = PP;
    handle->DIR = DIR;
    /* 给一个固定转矩，让电机静止 */
    FOC_SetTorque(handle, 3.0f, FOC_3PI_2);
    delay(3000);
    /* 读取当前电角度作为零点 */
    handle->zero_electric_angle = FOC_ElectricalAngle(handle);
    /* 停止输出 */
    FOC_SetTorque(handle, 0, FOC_3PI_2);
    Serial.print("零点电角度：");
    Serial.println(handle->zero_electric_angle, 4);
}

/* ===================== 获取机械角度 ===================== */
float FOC_GetMechanicalAngle(void)
{
    return AS5600_GetAngle(&M0_as5600_handle);
}

/* ===================== 串口接收用户目标角度 ===================== */
float FOC_SerialReceiveTarget(void)
{
    static String received_chars;
    String command = "";
    int commaPosition;

    while (Serial.available()) {
        char inChar = (char)Serial.read();
        received_chars += inChar;

        if (inChar == '\n') {
            command = received_chars;
            commaPosition = command.indexOf('\n');
            if (commaPosition != -1) {
                g_motor_target = command.substring(0, commaPosition).toFloat();
                Serial.println(g_motor_target);
            }
            received_chars = "";
        }
    }

    return g_motor_target;
}

/**/
//=================PID 设置函数=================
//速度PID
void FOC_M0_SET_VEL_PID(float P,float I,float D,float ramp,float limit)   //M0角度环PID设置
{
  M0_speed_pid_controller.P=P;
  M0_speed_pid_controller.I=I;
  M0_speed_pid_controller.D=D;
  M0_speed_pid_controller.output_ramp=ramp;
  M0_speed_pid_controller.limit=limit;
}
//角度PID
void FOC_M0_SET_ANGLE_PID(float P,float I,float D,float ramp,float limit)   //M0角度环PID设置
{
  M0_angle_pid_controller.P=P;
  M0_angle_pid_controller.I=I;
  M0_angle_pid_controller.D=D;
  M0_angle_pid_controller.output_ramp=ramp;
  M0_angle_pid_controller.limit=limit;
}
//电流PID
void FOC_M0_SET_CURRENT_PID(float P,float I,float D,float ramp) //M0电流环PID设置
{
  M0_current_pid_controller.P=P;
  M0_current_pid_controller.I=I;
  M0_current_pid_controller.D=D;
  M0_current_pid_controller.output_ramp=ramp;
}


//M0速度PID更新接口
float FOC_M0_VEL_PID_UPDATE(float error)   //M0速度环
{
    return PID_Update(&M0_speed_pid_controller, error);
}
//M0角度PID更新接口
float FOC_M0_ANGLE_PID_UPDATE(float error)
{
    return PID_Update(&M0_angle_pid_controller, error);
}

//M0电流PID更新接口
float FOC_M0_CURRENT_PID_UPDATE(float error)
{
    return PID_Update(&M0_current_pid_controller, error);
}

/* ===================== 低通滤波 =====================*/
//无滤波
//float FOC_M0_GetVelocity()
//{
//  return DIR*S0.getVelocity();
//}

//速度接口有滤波
float FOC_M0_GetVelocity()
{
    //获取速度数据并滤波
    float M0_Speed = AS5600_GetVelocity(&M0_as5600_handle);
    float M0_SpeedFilter = LowPassFilter_Update(&M0_speed_lowpass_filter, M0_Speed);
    return M0_SpeedFilter;   //考虑方向
}

//Iq接口滤波
float FOC_M0_GetIqCurrent()
{
    /* 计算Iq */
    g_M0_Iq_Current = FOC_calc_Iq(M0_current_value.current_a, M0_current_value.current_b, FOC_ElectricalAngle(&g_foc_handle));
    //滤波
    g_M0_Iq_Current = LowPassFilter_Update(&M0_Iqcurrent_lowpass_filter, g_M0_Iq_Current);
    return g_M0_Iq_Current;   //考虑方向
}


/* ===================== 电流值换算到Iq,Id =====================*/
#define _1_SQRT3 0.57735026919f    // 1/√3
#define _2_SQRT3 1.15470053838f    // 2/√3

/**
 * @brief 计算 q 轴电流 I_q
 * @param current_a    三相电流 A 相
 * @param current_b    三相电流 B 相
 * @param angle_el     电角度（弧度）
 * @return float       I_q 电流分量（扭矩电流）
 */
float FOC_calc_Iq(float current_a, float current_b, float angle_el)
{
    // ========== Step 1. Clarke 变换（abc -> αβ） ==========
    // 三相电流映射到两相静止坐标系 (α, β)
    // 数学公式：
    // I_alpha = I_a
    // I_beta  = (I_a + 2 * I_b) / √3
    float I_alpha = current_a;
    float I_beta  = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

    // ========== Step 2. Park 变换（αβ -> dq） ==========
    // 将 αβ 坐标系旋转到转子磁场坐标系 (d, q)
    // 数学公式：
    // I_d =  I_alpha * cos(θe) + I_beta * sin(θe)
    // I_q =  I_beta  * cos(θe) - I_alpha * sin(θe)
    float ct = cosf(angle_el);
    float st = sinf(angle_el);
    // float I_d = I_alpha * ct + I_beta * st; // 磁场分量（这里未使用）
    float I_q = I_beta * ct - I_alpha * st;   // 扭矩分量

    return I_q;
}


/* ===================== 辅助函数 ===================== */
/*
    角度归一化函数，它的作用是将任意弧度角归一化到 [0, 2π) 的范围内
*/
float _normalizeAngle(float angle)
{
    /*
        这里把 angle 对 2π 取余，得到角度在 [-2π, 2π) 的范围。
        如果 angle 是负数，余数也会是负数。
    */
    float a = fmod(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}

/**
 * @brief 获取M0电机的速度、角度和电流信息
 *
 * 该函数用于同步获取FOC控制所需的三个关键参数：
 * 1. 电机当前的电角度（用于坐标变换）
 * 2. 电机当前的转速（用于速度环控制）
 * 3. 电机当前的相电流（用于电流环控制）
 *
 * 这些参数将用于FOC算法的闭环控制计算
 */
void FOC_M0_Get_Angle_Velocity_Current(void)
{
    // 获取电机当前角度（单位：弧度）
    my_foc_motorParm.M0_motor_angle = FOC_GetMechanicalAngle();
    // 获取电机当前速度（单位：弧度/秒）
    // 经过低通滤波处理，消除噪声，提供平滑的速度反馈
    my_foc_motorParm.M0_motor_velocity = FOC_M0_GetVelocity();
    // 读取电机当前相电流（单位：安培）
    // 通过电流传感器获取实时的U、V相电流，用于电流闭环控制
    CurrSense_ReadCurrents(&M0_current_value);
    my_foc_motorParm.M0_motor_current =  FOC_M0_GetIqCurrent();
}

/* =====  测试函数  ====*/
/*  1️⃣ 角度闭环 + 速度闭环（Position + Velocity Loop） */
void DFOC_M0_Set_Velocity_Angle(float target_angle_rad)
{
    //设置速度环PID
    //FOC_M0_SET_VEL_PID(0.005,0.00,0,0);

    // 当前机械角度（弧度）
    float current_angle = FOC_GetMechanicalAngle();
    // 角度误差（弧度）
    float angle_error = target_angle_rad - current_angle;
    // 角度PID输出目标速度（弧度/秒）
    float target_velocity = FOC_M0_ANGLE_PID_UPDATE(angle_error);
    // 当前速度（弧度/秒，低通滤波）
    float current_velocity = FOC_M0_GetVelocity();
    // 速度误差，速度PID输出电压/力矩
    float torque = FOC_M0_VEL_PID_UPDATE(target_velocity - current_velocity);
    // 输出到电机
    #if USE_VOLTAGE_CONTROL
    FOC_SetTorque(&g_foc_handle, torque, FOC_ElectricalAngle(&g_foc_handle));           //电压直接作用到电机
    #else
    DFOC_M0_setTorque(torque);                                                          //电流环来控制力矩
    #endif
}

/*2️⃣ 速度闭环（Velocity Loop）*/
void DFOC_M0_SetVelocity(float target_velocity_rad_s)
{
    // 当前速度（弧度/秒，低通滤波）
    float current_velocity = FOC_M0_GetVelocity();
    // 速度误差，速度PID输出电压/力矩
    float torque = FOC_M0_VEL_PID_UPDATE(target_velocity_rad_s - current_velocity);
    // 输出到电机
    #if USE_VOLTAGE_CONTROL
    FOC_SetTorque(&g_foc_handle, torque, FOC_ElectricalAngle(&g_foc_handle));           //电压直接作用到电机
    #else
    DFOC_M0_setTorque(torque);                                                          //电流环来控制力矩
    #endif
}


/*3️⃣ 角度闭环 / 力位控制（Position / Force-Angle Loop）*/
void DFOC_M0_Set_Force_Angle(float target_angle_rad)
{
    // 当前机械角度（弧度）
    float current_angle = FOC_GetMechanicalAngle();
    // 角度误差（弧度）
    float angle_error = target_angle_rad - current_angle;
    // 角度PID输出电压/力矩
    float torque = FOC_M0_ANGLE_PID_UPDATE(angle_error);
    // 输出到电机
    #if USE_VOLTAGE_CONTROL
    FOC_SetTorque(&g_foc_handle, torque, FOC_ElectricalAngle(&g_foc_handle));           //电压直接作用到电机
    #else
    DFOC_M0_setTorque(torque);                                                          //电流环来控制力矩
    #endif
}

/*4️⃣ 开环力矩 / 电压输出（Torque Control）*/
void DFOC_M0_SetTorqueVoltage(float torque)
{
    // 直接输出电压/力矩
    #if USE_VOLTAGE_CONTROL
    FOC_SetTorque(&g_foc_handle, torque, FOC_ElectricalAngle(&g_foc_handle));           //电压直接作用到电机
    #else
    DFOC_M0_setTorque(torque);                                                          //电流环来控制力矩
    #endif
}


/*4️⃣ 开环力矩 / 电流输出（Torque Control）*/
void DFOC_M0_setTorque(float Target)
{
    FOC_SetTorque(&g_foc_handle, FOC_M0_CURRENT_PID_UPDATE(Target-FOC_M0_GetIqCurrent()), FOC_ElectricalAngle(&g_foc_handle));
}
