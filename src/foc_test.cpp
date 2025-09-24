#include "foc_test.h"
#include <Arduino.h>
#include <math.h>

/* FOC 句柄 */
static FOC_Handle_t g_foc_handle;

/* ===================== 初始化示例 ===================== */
void FOC_Example_Init(float voltage_supply)
{
    /* 初始化 FOC */
    FOC_Init(&g_foc_handle, voltage_supply);
    /* 零点校准，假设极对数 PP=7，方向 DIR=1 */
    FOC_AlignSensor(&g_foc_handle, 7, 1);
    Serial.println("FOC 初始化完成");
}

/* ===================== 开环速度测试 ===================== */
void FOC_OpenLoopSpeedTest(float speed_rad_s, uint32_t duration_ms)
{
    uint32_t start_time = millis();
    float angle_el = 0.0f;

    while (millis() - start_time < duration_ms)
    {
        /* 角度累加 */
        angle_el += speed_rad_s * 0.001f; // 每循环假设 1ms
        angle_el = _normalizeAngle(angle_el);
        /* 输出固定转矩，角度累加形成开环速度控制 */
        FOC_SetTorque(&g_foc_handle, 3.0f, angle_el);
        delay(1); // 延时 1ms 模拟定时控制
    }

    /* 停止输出 */
    FOC_SetTorque(&g_foc_handle, 0, angle_el);
    Serial.println("开环速度测试完成");
}

/* ===================== 位置闭环测试 ===================== */
void FOC_PositionClosedLoopTest(float target_angle_rad, uint32_t duration_ms)
{
    /* 简单 P 控制器 */
    float Kp = 0.133f;
    uint32_t start_time = millis();

    while (millis() - start_time < duration_ms)
    {
        float current_angle = FOC_GetMechanicalAngle();  // 获取累计旋转角
        float error = target_angle_rad - current_angle;
        float Uq = Kp * error;                           // 简单 P 控制器

        /* 限制最大电压 */
        if (Uq > g_foc_handle.voltage_supply / 2) Uq = g_foc_handle.voltage_supply / 2;
        if (Uq < -g_foc_handle.voltage_supply / 2) Uq = -g_foc_handle.voltage_supply / 2;

        float angle_el = FOC_ElectricalAngle(&g_foc_handle);
        FOC_SetTorque(&g_foc_handle, Uq, angle_el);

        Serial.print("当前角度(rad): ");
        Serial.println(current_angle, 4);

        delay(10); // 控制周期 10ms
    }

    /* 停止输出 */
    FOC_SetTorque(&g_foc_handle, 0, FOC_ElectricalAngle(&g_foc_handle));
    Serial.println("位置闭环测试完成");
}
