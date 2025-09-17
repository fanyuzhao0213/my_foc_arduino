#include "pwm.h"
#include <Arduino.h>

// PWM 通道号
int channel1 = 0;
int channel2 = 1;
int channel3 = 2;

// PWM 引脚
static int pwmPin1 = -1;
static int pwmPin2 = -1;
static int pwmPin3 = -1;

void pwm_init(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint32_t freq, uint8_t resolution) {
    pwmPin1 = pin1;
    pwmPin2 = pin2;
    pwmPin3 = pin3;

    // 配置 PWM 通道
    ledcSetup(channel1, freq, resolution);
    ledcSetup(channel2, freq, resolution);
    ledcSetup(channel3, freq, resolution);

    // 绑定引脚
    ledcAttachPin(pwmPin1, channel1);
    ledcAttachPin(pwmPin2, channel2);
    ledcAttachPin(pwmPin3, channel3);

    Serial.println("[PWM] Init complete");
}

void pwm_set_duty(uint8_t channel, uint8_t duty)
{
    ledcWrite(channel, duty);
}
