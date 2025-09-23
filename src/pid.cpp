#include "pid.h"
#include <Arduino.h>  // 使用 micros() 获取时间戳

/**
 * @brief 初始化 PID 控制器
 */
void PID_Init(PIDController *pid, float P, float I, float D, float ramp, float limit)
{
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;
    pid->limit = limit;

    pid->error_prev = 0.0f;
    pid->integral_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->timestamp_prev = micros(); // 记录当前时间戳
}

/**
 * @brief PID 控制计算
 *
 * 使用 **Tustin 积分法** 处理积分项，避免传统矩形积分带来的精度问题。
 */
float PID_Update(PIDController *pid, float error)
{
    // ---------- 1. 计算时间间隔 ----------
    uint32_t timestamp_now = micros();
    float Ts = (timestamp_now - pid->timestamp_prev) * 1e-6f; // 转换为秒

    // 防止计算异常：如果时间间隔过小或过大，使用默认采样周期 1ms
    if (Ts <= 0.0f || Ts > 0.5f) {
        Ts = 1e-3f;
    }

    // ---------- 2. P 环 ----------
    float proportional = pid->P * error;

    // ---------- 3. I 环（Tustin 积分法） ----------
    float integral = pid->integral_prev + pid->I * Ts * 0.5f * (error + pid->error_prev);

    // 对积分项进行限幅，防止积分饱和
    integral = PID_Constrain(integral, -pid->limit, pid->limit);

    // ---------- 4. D 环 ----------
    float derivative = pid->D * (error - pid->error_prev) / Ts;

    // ---------- 5. PID 输出计算 ----------
    float output = proportional + integral + derivative;

    // 输出限幅
    output = PID_Constrain(output, -pid->limit, pid->limit);

    // ---------- 6. 输出变化速率限制 ----------
    if (pid->output_ramp > 0.0f) {
        float output_rate = (output - pid->output_prev) / Ts;
        if (output_rate > pid->output_ramp) {
            output = pid->output_prev + pid->output_ramp * Ts;
        } else if (output_rate < -pid->output_ramp) {
            output = pid->output_prev - pid->output_ramp * Ts;
        }
    }

    // ---------- 7. 保存状态 ----------
    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
    pid->timestamp_prev = timestamp_now;

    return output;
}

/**
 * @brief 重置 PID 控制器状态
 */
void PID_Reset(PIDController *pid)
{
    pid->error_prev = 0.0f;
    pid->integral_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->timestamp_prev = micros();
}
