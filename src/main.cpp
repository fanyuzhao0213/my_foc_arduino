#include <Arduino.h>
#include "foc.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "serial.h"

// ==================== 共享PID和低通滤波器 ====================
// 速度环PID
PIDController sharedVelPID = {
    2.0f,      // P
    0.1f,      // I
    0.0f,      // D
    10000.0f,  // 输出斜率限制
    6.0f       // 电压输出限制
};

// 角度环PID
PIDController sharedAnglePID = {
    2.0f,      // P
    0.0f,      // I
    0.0f,      // D
    10000.0f,  // 输出斜率限制
    100.0f     // 速度输出限制
};

// 速度低通滤波器
LowPassFilter sharedFilter;

// ==================== 电机实例 ====================
MotorFOC_t motor0;  // 电机0
MotorFOC_t motor1;  // 电机1

void setup() {
    Serial.begin(115200);  // 初始化串口打印

    // 初始化低通滤波器
    LowPassFilter_Init(&sharedFilter, 0.01f);  // Tf = 10ms

    // -------------------- 初始化电机 --------------------
    // 参数: 电机实例, PWM引脚A/B/C, 编码器IIC引脚,极对数, 方向, 速度PID, 角度PID, 滤波器, 供电电压
    MotorFOC_Init(&motor0, 32,33,25,19,18, 7,1, &sharedVelPID, &sharedAnglePID, &sharedFilter, 12.0f);
    MotorFOC_Init(&motor1, 26,27,14,23,5, 7,-1, &sharedVelPID, &sharedAnglePID, &sharedFilter, 12.0f);

    // -------------------- 硬件初始化 --------------------
    motor0.begin(&motor0);   // 初始化PWM和传感器
    motor1.begin(&motor1);

    // -------------------- 零点校准 --------------------
    motor0.alignSensor(&motor0);  // 电机0归零
    motor1.alignSensor(&motor1);  // 电机1归零

    // -------------------- 设置控制模式 --------------------
    motor0.mode = MOTOR_DOUBLE_LOOP; // 电机0使用双闭环（角度+速度）
    motor1.mode = MOTOR_SPEED_LOOP;  // 电机1使用单环速度
}

void loop() {
    // ==================== 更新电机控制 ====================
    motor0.update(&motor0);  // 电机0 PID计算并输出PWM
    motor1.update(&motor1);  // 电机1 PID计算并输出PWM

    // ==================== 串口动态调节 ====================
    // 非阻塞串口读取数字
    if (serial_process()) {  // serial_process() 返回 true 表示接收到有效命令
        float motor_target = serial_getMotorTarget(); // 获取串口输入的目标值
        Serial.print("串口目标：");
        Serial.println(motor_target);
        // 可根据需要写入 motor0.target_angle 或 motor0.target_velocity
        // 例如: motor0.target_angle = motor_target;
    }

    delay(10);  // 10ms循环，非阻塞控制
}
