#ifndef CURRt_SENSOR_H
#define CURRt_SENSOR_H

#include <Arduino.h>

// ------------------------- 宏定义 -------------------------
#define ADC_VOLTAGE      3.3f       // ADC 电压参考
#define ADC_RESOLUTION   4095.0f    // ADC 分辨率
#define ADC_CONV         (ADC_VOLTAGE / ADC_RESOLUTION) // ADC计数转电压

#define NOT_SET          -12345.0f
#define IS_SET(a)        ((a) != NOT_SET)

// ------------------------- 电流采集结构体 -------------------------
typedef struct {
    int pinA;       // A相 ADC 引脚
    int pinB;       // B相 ADC 引脚
    int pinC;       // C相 ADC 引脚，可选
    float shunt_resistor;  // 分流电阻值 (Ω)
    float amp_gain;        // 放大器增益
    float volts_to_amps;   // 电压转电流比例

    float gain_a;   // A相转换系数
    float gain_b;   // B相转换系数
    float gain_c;   // C相转换系数

    float offset_a; // A相零偏
    float offset_b; // B相零偏
    float offset_c; // C相零偏

    float current_a; // 实时电流
    float current_b;
    float current_c;

} CurrSense_t;

// ------------------------- 函数声明 -------------------------

// 初始化结构体
void CurrSense_Init(CurrSense_t* cs, int motorNum);

// 配置 ADC 引脚
void CurrSense_ConfigADC(CurrSense_t* cs);

// 校准零偏
void CurrSense_Calibrate(CurrSense_t* cs);

// 读取单个 ADC 电压
float CurrSense_ReadADCVoltage(CurrSense_t* cs, int pin);

// 获取三相电流
void CurrSense_ReadCurrents(CurrSense_t* cs);

#endif

