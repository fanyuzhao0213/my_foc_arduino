#ifndef FOC_TEST_H
#define FOC_TEST_H

#include "foc.h"
#include "AS5600.h"

/* ===================== 函数声明 ===================== */
void FOC_Example_Init(float voltage_supply);
void FOC_OpenLoopSpeedTest(float speed_rad_s, uint32_t duration_ms);
void FOC_PositionClosedLoopTest(float target_angle_rad, uint32_t duration_ms);

#endif
