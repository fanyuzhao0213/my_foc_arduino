#include <Arduino.h>
#include "foc.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "serial.h"
#include "foc_test.h"


#define MOTOR_ENBALE_PIN            12
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

int print_count = 0;
void loop() {

    FOC_M0_Get_Angle_Velocity_Current();

    print_count++;
    if(print_count >= 30)
    {
        print_count = 0;
        Serial.printf("%f,%f,%f\n", my_foc_motorParm.M0_motor_angle,my_foc_motorParm.M0_motor_velocity,my_foc_motorParm.M0_motor_current);
    }

    //力位（加入电流环后）
    #if 0
    FOC_M0_SET_ANGLE_PID(0.5,0,0.003,100000,0.1);   //这个限制了下一级电流的上限
    FOC_M0_SET_CURRENT_PID(1.25,50,0,100000);
    DFOC_M0_Set_Force_Angle(serial_getMotorTarget());
    #endif

    //速度（加入电流环后）
    #if 0
    FOC_M0_SET_VEL_PID(3,2,0,100000,0.5);           //这个限制了下一级电流的上限
    FOC_M0_SET_CURRENT_PID(0.5,50,0,100000);
    DFOC_M0_SetVelocity(serial_getMotorTarget());
    #endif

    //位置-速度-力（加入电流环后）
    #if 0
    FOC_M0_SET_ANGLE_PID(1,0,0,100000,30);          //这个限制了下一级速度的上限
    FOC_M0_SET_VEL_PID(0.02,1,0,100000,0.5);        //这个限制了下一级电流的上限
    FOC_M0_SET_CURRENT_PID(5,200,0,100000);
    DFOC_M0_Set_Velocity_Angle(serial_getMotorTarget());
    #endif

    //电流力矩
    #if 0
    FOC_M0_SET_CURRENT_PID(5,200,0,100000);
    DFOC_M0_setTorque(serial_getMotorTarget());
    #endif

    #if 0
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
    #endif
    serial_process();               //只是用来接收串口发送过来的target
}
