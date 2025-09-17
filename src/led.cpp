#include "led.h"
#include "Arduino.h"
#include <stdarg.h>

static int led_pin = -1;   // 保存 LED 引脚号
static int led_state = LOW; // 当前 LED 状态

// ===================== 调试打印 =====================
void led_debug(const char *fmt, ...) {
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    Serial.print(buffer);   // 使用 Arduino 串口输出
}

// ===================== LED 控制函数 =====================
void led_init(int pin) {
    led_pin = pin;
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LOW);
    led_debug("[LED] Init pin = %d\n", led_pin);
}

void led_on(void) {
    if (led_pin >= 0) {
        digitalWrite(led_pin, HIGH);
        led_state = HIGH;
    }
}

void led_off(void) {
    if (led_pin >= 0) {
        digitalWrite(led_pin, LOW);
        led_state = LOW;
    }
}

void led_toggle(void) {
    if (led_pin >= 0) {
        led_state = !led_state;
        digitalWrite(led_pin, led_state);
    }
}
