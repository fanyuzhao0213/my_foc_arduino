#include <Arduino.h>
#include "foc.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "serial.h"
#include "foc_test.h"


#define MOTOR_ENBALE_PIN            12
// ==================== 共享PID和低通滤波器 ====================
void my_motor_enable_config(void)
{
    pinMode(MOTOR_ENBALE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENBALE_PIN, HIGH);
}
void setup() {
    Serial.begin(115200);               // 初始化串口打印
    my_motor_enable_config();           // 电机使能引脚
    delay(1000);
    /* 初始化 FOC + AS5600 */
    FOC_Example_Init(12.0f); // 12V 电源
    delay(500);
}

void loop() {
    /* 开环速度测试 1rad/s，持续 5秒 */
    FOC_OpenLoopSpeedTest(5.0f, 5000);
    delay(2000);
    FOC_OpenLoopSpeedTest(5.0f, 10000);
    delay(2000);
    FOC_OpenLoopSpeedTest(5.0f, 3000);
    delay(2000);
    FOC_OpenLoopSpeedTest(5.0f, 5000);
    delay(2000);

    /* 位置闭环测试，目标 90° (PI/2 rad)，持续 3秒 */
    FOC_PositionClosedLoopTest(PI/2, 3000);
    delay(2000);
    FOC_PositionClosedLoopTest(PI, 3000);
    delay(2000);
    FOC_PositionClosedLoopTest(2*PI, 3000);
    delay(2000);
    FOC_PositionClosedLoopTest(5*PI, 3000);
    delay(2000);
    FOC_PositionClosedLoopTest(0, 3000);
    delay(2000);

    delay(10);  // 10ms循环，非阻塞控制
}
