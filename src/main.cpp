#include <Arduino.h>
#include "led.h"
#include "serial.h"
#include "as5600.h"
#include "foc.h"


#define LED_PIN       22

int Sensor_DIR=-1;    //传感器方向
int Motor_PP=7;    //电机极对数

void setup() {
    serial_init(115200); // 初始化串口

    led_init(LED_PIN);   // 初始化 LED
    led_on();            // 点亮 LED

    DFOC_Vbus(12.0);   //设定驱动器供电电压
    DFOC_alignSensor(Motor_PP, Sensor_DIR);
}

void loop() {
    static uint8_t A=80;
    static uint8_t B=50;
    static uint8_t C=20;
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
    #if 0
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

    //输出角度值
    float Kp=0.133;
    float Sensor_Angle=DFOC_M0_Angle();
    setTorque(Kp*(serial_getMotorTarget()-Sensor_DIR*Sensor_Angle)*180/PI,_electricalAngle());   //位置闭环
    //setTorque(serial_motor_target(),_electricalAngle());   //电压力矩

    A= (A+1)%100;
    B= (B+1)%100;
    C= (C+1)%100;
    Serial.printf("<demo>: %d,%d,%d\n",A,B,C);
    delay(100);
}

