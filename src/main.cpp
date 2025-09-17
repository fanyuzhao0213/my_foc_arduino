#include <Arduino.h>
#include "led.h"
#include "serial.h"
#include "as5600.h"
#include "foc.h"


#define LED_PIN       22

void setup() {
    serial_init(115200); // 初始化串口

    led_init(LED_PIN);   // 初始化 LED
    led_on();            // 点亮 LED

    DFOC_Vbus(12.0); // 电源电压6V示例
    DFOC_alignSensor(1, 1);
}

void loop() {
    // led_toggle();        // 每次循环 LED 状态切换
    // delay(1000);         // 1 秒闪烁一次
    #if 0
    // PWM 占空比循环
    for (int duty = 0; duty <= 255; duty += 5) {
        pwm_set_duty(channel1, duty);
        pwm_set_duty(channel2, duty);
        pwm_set_duty(channel3, duty);
        delay(20);
    }
    #endif
    #if 1
    // 非阻塞接收数字
    if (serial_process()) {
        float target = serial_getMotorTarget();
        // TODO: 将 target 用于电机控制
        Serial.print("Do something with target: ");
        Serial.println(target);

        // 示例：根据目标角度设置电机转矩
        setTorque(target, 0);
    }
    #endif
}

