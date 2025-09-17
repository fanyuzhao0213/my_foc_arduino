#include "serial.h"

//----------------- 内部变量 -----------------
#define SERIAL_BUFFER_SIZE 64
static char buffer[SERIAL_BUFFER_SIZE]; // 存放接收数据
static uint8_t buf_index = 0;           // 当前写入位置
static float motor_target = 0.0f;       // 最新解析的电机目标

//----------------- 对外接口 -----------------

void serial_init(uint32_t baudrate) {
    Serial.begin(baudrate);
    while (!Serial); // 等待串口就绪
    Serial.println("Serial module ready");
}

/**
 * @brief 非阻塞串口接收，遇到 '\n' 转为浮点数
 * @return true 如果收到完整命令
 */
bool serial_process(void) {
    bool newData = false;

    while (Serial.available()) {
        char inChar = (char)Serial.read();

        // 判断结束符或缓冲区满
        if (inChar == '\n' || buf_index >= SERIAL_BUFFER_SIZE - 1) {
            buffer[buf_index] = '\0'; // 字符串结束
            buf_index = 0;            // 重置索引

            // 转换为浮点数
            motor_target = atof(buffer);
            Serial.print("Received target: ");
            Serial.println(motor_target);

            newData = true;           // 标记收到新命令
        }
        else if (inChar != '\r') {    // 忽略回车
            buffer[buf_index++] = inChar;
        }
    }

    return newData;
}

float serial_getMotorTarget(void) {
    return motor_target;
}
