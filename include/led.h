#ifndef LED_H
#define LED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 LED 引脚
void led_init(int pin);

// 点亮 LED
void led_on(void);

// 熄灭 LED
void led_off(void);

// LED 取反（闪烁）
void led_toggle(void);

// 调试打印接口
void led_debug(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
