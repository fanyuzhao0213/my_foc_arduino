#include "pid.h"

/**
 * @brief 构造函数，初始化 PID 参数和中间变量
 */
PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)
    , limit(limit)
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
{
    timestamp_prev = micros(); // 记录当前时间，单位：us
}

/**
 * @brief PID 控制计算
 */
float PIDController::operator()(float error) {
    // ---------- 1. 计算时间间隔 ----------
    unsigned long timestamp_now = micros();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f; // us → s
    if (Ts <= 0 || Ts > 0.5f) {
        Ts = 1e-3f; // 防止计算异常，默认采样周期 1ms
    }

    // ---------- 2. P 环 ----------
    float proportional = P * error;

    // ---------- 3. I 环（积分） ----------
    // 使用 **Tustin 积分**（梯形法）提高精度
    float integral = integral_prev + I * Ts * 0.5f * (error + error_prev);

    // 对积分项限幅，避免积分饱和（windup）
    integral = _constrain(integral, -limit, limit);

    // ---------- 4. D 环（微分） ----------
    float derivative = D * (error - error_prev) / Ts;

    // ---------- 5. PID 三项求和 ----------
    float output = proportional + integral + derivative;

    // 输出限幅
    output = _constrain(output, -limit, limit);

    // ---------- 6. 输出变化速率限幅 ----------
    if (output_ramp > 0) {
        float output_rate = (output - output_prev) / Ts;
        if (output_rate > output_ramp) {
            output = output_prev + output_ramp * Ts;
        } else if (output_rate < -output_ramp) {
            output = output_prev - output_ramp * Ts;
        }
    }

    // ---------- 7. 保存状态 ----------
    integral_prev = integral;       // 保存积分值
    output_prev = output;           // 保存输出
    error_prev = error;             // 保存误差
    timestamp_prev = timestamp_now; // 保存时间戳

    return output;
}

/**
 * @brief 重置 PID 控制器状态
 */
void PIDController::reset() {
    error_prev = 0.0f;
    output_prev = 0.0f;
    integral_prev = 0.0f;
    timestamp_prev = micros();
}
