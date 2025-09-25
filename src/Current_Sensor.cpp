#include "Current_Sensor.h"
#include <Arduino.h>
#include "serial.h"

// ------------------------- 初始化函数 -------------------------
void CurrSense_Init(CurrSense_t* cs, int motorNum) {
    // 根据电机编号选择引脚
    if (motorNum == 0) {
        cs->pinA = 39;
        cs->pinB = 36;
        cs->pinC = NOT_SET; // 如果不使用C相
    } else if (motorNum == 1) {
        cs->pinA = 35;
        cs->pinB = 34;
        cs->pinC = NOT_SET;
    }

    cs->shunt_resistor = 0.01f;
    cs->amp_gain = 50.0f;

    // 电压转电流比例
    cs->volts_to_amps = 1.0f / (cs->shunt_resistor * cs->amp_gain);

    // 相位增益，考虑接线方向
    /*
        这个符号 (*1 或 *-1) 不是固定的理论值，而是 根据实际接线和电机转矩方向测试得到的。
        原因是：
        INA240 测量的只是电压降，正负取决于你接线的方向。
        三相电机的电流方向和 FOC 算法中定义的 dq 正方向要匹配，否则正电流反而会产生负转矩。
        所以通常的做法是：
        电机静止，给一个小电流或小电压锁定转子。
        读取各相 ADC 值。
        判断哪个方向的电流是正的，哪个是负的。
        对照 FOC 算法，设置 *1 或 *-1 使得三相电流方向统一。
        简单来说：这是一个“实测校准”，保证软件里的正电流对应正转矩。
    */
    cs->gain_a = -cs->volts_to_amps;
    cs->gain_b = -cs->volts_to_amps;
    cs->gain_c = cs->volts_to_amps;

    // 零偏初始化
    cs->offset_a = 0;
    cs->offset_b = 0;
    cs->offset_c = 0;

    cs->current_a = 0;
    cs->current_b = 0;
    cs->current_c = 0;

    // 配置 ADC
    CurrSense_ConfigADC(cs);
    // 校准零偏
    CurrSense_Calibrate(cs);
}

// ------------------------- 配置 ADC -------------------------
void CurrSense_ConfigADC(CurrSense_t* cs) {
    pinMode(cs->pinA, INPUT);
    pinMode(cs->pinB, INPUT);
    if (IS_SET(cs->pinC)) pinMode(cs->pinC, INPUT);
}

// ------------------------- 校准零偏 -------------------------
void CurrSense_Calibrate(CurrSense_t* cs) {
    const int rounds = 1000;
    float sum_a = 0, sum_b = 0, sum_c = 0;

    for (int i = 0; i < rounds; i++) {
        sum_a += CurrSense_ReadADCVoltage(cs, cs->pinA);
        sum_b += CurrSense_ReadADCVoltage(cs, cs->pinB);
        if (IS_SET(cs->pinC)) sum_c += CurrSense_ReadADCVoltage(cs, cs->pinC);
        delay(1);
    }

    cs->offset_a = sum_a / rounds;
    cs->offset_b = sum_b / rounds;
    if (IS_SET(cs->pinC)) cs->offset_c = sum_c / rounds;

    Serial.printf("Current Sensor Calibrated,cs->offset_a:%d cs->offset_b:%d\n",cs->offset_a,cs->offset_b);
}

// ------------------------- 读取 ADC 电压 -------------------------
float CurrSense_ReadADCVoltage(CurrSense_t* cs, int pin) {
    uint32_t raw = analogRead(pin);
    return raw * ADC_CONV;
}

// ------------------------- 获取三相电流 -------------------------
void CurrSense_ReadCurrents(CurrSense_t* cs) {
    cs->current_a = (CurrSense_ReadADCVoltage(cs, cs->pinA) - cs->offset_a) * cs->gain_a;
    cs->current_b = (CurrSense_ReadADCVoltage(cs, cs->pinB) - cs->offset_b) * cs->gain_b;
    cs->current_c = IS_SET(cs->pinC) ? (CurrSense_ReadADCVoltage(cs, cs->pinC) - cs->offset_c) * cs->gain_c : 0.0f;
}

