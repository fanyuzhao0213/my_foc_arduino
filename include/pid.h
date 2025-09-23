#ifndef PID_H
#define PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID 控制器结构体
 */
typedef struct {
    float P;                  // 比例系数
    float I;                  // 积分系数
    float D;                  // 微分系数
    float output_ramp;        // 输出变化速率限制 (单位: 每秒最大变化量)
    float limit;              // 输出限幅

    float error_prev;         // 上一次误差
    float integral_prev;      // 积分项上次值
    float output_prev;        // 上一次输出值

    uint32_t timestamp_prev;  // 上一次计算的时间戳(us)
} PIDController;

/**
 * @brief 初始化 PID 控制器
 *
 * @param pid PID 控制器指针
 * @param P 比例系数
 * @param I 积分系数
 * @param D 微分系数
 * @param ramp 输出变化速率限制
 * @param limit 输出限幅
 */
void PID_Init(PIDController *pid, float P, float I, float D, float ramp, float limit);

/**
 * @brief PID 控制计算
 *
 * @param pid PID 控制器指针
 * @param error 当前误差
 * @return float PID 输出
 */
float PID_Update(PIDController *pid, float error);

/**
 * @brief 重置 PID 控制器状态
 *
 * @param pid PID 控制器指针
 */
void PID_Reset(PIDController *pid);

/**
 * @brief 安全限幅函数
 */
static inline float PID_Constrain(float value, float min, float max) {
    return (value < min) ? min : (value > max) ? max : value;
}

#ifdef __cplusplus
}
#endif

#endif // PID_H
