#ifndef PWM_H
#define PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// PWM 通道号
extern int channel1;
extern int channel2;
extern int channel3;

// 初始化 3 路 PWM
void pwm_init(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint32_t freq, uint8_t resolution);

// 设置 PWM 占空比
void pwm_set_duty(uint8_t channel, uint8_t duty);

#ifdef __cplusplus
}
#endif

#endif
