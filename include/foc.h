#ifndef FOC_H
#define FOC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 PWM 和传感器
 * @param power_supply 电源电压（V）
 */
void DFOC_Vbus(float power_supply);

/**
 * @brief 设置电机电流向量的 q 分量，从而控制转矩
 * @param Uq q 轴电压
 * @param angle_el 电角度
 */
void setTorque(float Uq, float angle_el);

/**
 * @brief 将角度归一化到 [0, 2*PI]
 * @param angle 待归一化角度（弧度）
 * @return 归一化后的角度
 */
float _normalizeAngle(float angle);

/**
 * @brief 通过电机静止校准得到零电角度
 * @param _PP 极对数
 * @param _DIR 转向（1正向，-1反向）
 */
void DFOC_alignSensor(int _PP, int _DIR);

/**
 * @brief 获取电机电角度（弧度）
 * @return 电角度
 */
float _electricalAngle(void);

/**
 * @brief 获取电机机械角度（弧度）
 * @return 机械角度
 */
float DFOC_M0_Angle(void);

#ifdef __cplusplus
}
#endif

#endif
