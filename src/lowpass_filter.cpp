#include "lowpass_filter.h"
#include <Arduino.h>  // 使用 micros() 获取时间戳

/**
 * @brief 初始化低通滤波器
 */
void LowPassFilter_Init(LowPassFilter *filter, float time_constant)
{
    filter->Tf = time_constant;
    filter->y_prev = 0.0f;
    filter->timestamp_prev = micros(); // 记录初始化时间戳
}

/**
 * @brief 执行一次低通滤波计算
 */
float LowPassFilter_Update(LowPassFilter *filter, float x)
{
    uint32_t timestamp = micros();

    // 计算当前采样周期 dt (单位: 秒)
    float dt = (timestamp - filter->timestamp_prev) * 1e-6f;

    // ---------- 防止异常 dt ----------
    // 如果 dt < 0，则设置为 1ms，避免计算异常
    if (dt < 0.0f) {
        dt = 1e-3f;
    }
    // 如果 dt > 0.3s，说明长时间没有调用，直接重置输出
    else if (dt > 0.3f) {
        filter->y_prev = x;
        filter->timestamp_prev = timestamp;
        return x;
    }

    // ---------- 一阶低通滤波核心 ----------
    // α = Tf / (Tf + dt)
    float alpha = filter->Tf / (filter->Tf + dt);

    // 输出计算： y = α*y_prev + (1-α)*x
    float y = alpha * filter->y_prev + (1.0f - alpha) * x;

    // ---------- 保存状态 ----------
    filter->y_prev = y;
    filter->timestamp_prev = timestamp;

    return y;
}


