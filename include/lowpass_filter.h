#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include <stdint.h>

/**
 * @brief 低通滤波器结构体
 *
 * 一阶低通滤波器公式：
 * y[n] = α * y[n-1] + (1 - α) * x[n]
 * 其中 α = Tf / (Tf + dt)
 */
typedef struct {
    float Tf;                  /**< 滤波时间常数，越大滤波越平滑 */
    float y_prev;              /**< 上一次滤波输出 */
    uint32_t timestamp_prev;   /**< 上一次调用时间（单位：us） */
} LowPassFilter;

/**
 * @brief 初始化低通滤波器
 * @param filter 滤波器句柄
 * @param time_constant 滤波时间常数 Tf
 */
void LowPassFilter_Init(LowPassFilter *filter, float time_constant);

/**
 * @brief 执行一次低通滤波计算
 * @param filter 滤波器句柄
 * @param x 当前输入值
 * @return 滤波后的输出值
 */
float LowPassFilter_Update(LowPassFilter *filter, float x);

#endif // LOWPASS_FILTER_H
