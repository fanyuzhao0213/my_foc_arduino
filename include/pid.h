#ifndef PID_H
#define PID_H

#include <Arduino.h>

/**
 * @brief 简单的约束函数，将值限制在 [low, high] 之间
 */
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

/**
 * @brief PID 控制器类
 *
 * 支持 P、I、D 三项计算，并加入了积分限幅、输出限幅、输出变化速率限幅（ramp 限幅）。
 * 控制算法使用 **Tustin 积分**，适合实时控制场合，例如电机控制、平衡小车等。
 */
class PIDController {
public:
    /**
     * @brief 构造函数
     * @param P      比例系数 (Proportional gain)
     * @param I      积分系数 (Integral gain)
     * @param D      微分系数 (Derivative gain)
     * @param ramp   输出变化速率限幅，单位：每秒最大变化量
     * @param limit  输出最大绝对值限幅
     */
    PIDController(float P, float I, float D, float ramp, float limit);

    /**
     * @brief PID 控制计算
     * @param error 当前误差 = 目标值 - 测量值
     * @return 计算后的 PID 输出
     */
    float operator()(float error);

    /**
     * @brief 重置 PID 控制器的内部状态
     *        包括积分项、上次输出、上次误差、时间戳
     */
    void reset();

private:
    // PID 参数
    float P;            ///< 比例系数
    float I;            ///< 积分系数
    float D;            ///< 微分系数
    float output_ramp;  ///< 输出变化速率限幅
    float limit;        ///< 输出绝对值限幅

    // 中间状态变量
    float error_prev;     ///< 上一次误差
    float output_prev;    ///< 上一次输出
    float integral_prev;  ///< 上一次积分项
    unsigned long timestamp_prev; ///< 上一次时间戳（微秒）
};

#endif // PID_H

