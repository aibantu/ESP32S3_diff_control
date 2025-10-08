// #ifndef WHEEL_SPEED_CTRL_H
// #define WHEEL_SPEED_CTRL_H
// #include <Arduino.h>
// #include "pid.h"
// #include "motor.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #ifdef __cplusplus
// extern "C" {
// #endif

// // 已移除外部模式枚举与全局模式变量，系统仅保留车轮速度控制任务。
// extern volatile uint32_t g_lastWheelCmdMs;
// extern volatile float wheelSpeedRefL;
// extern volatile float wheelSpeedRefR;

// // 任务句柄（可被外部用于删除/判断）
// extern TaskHandle_t wheelSpeedTaskHandle;

// // 任务函数（允许在外部重新创建）
// void WheelSpeedTask(void *arg);

// void WheelSpeedCtrl_Init(void);
// void WheelSpeedCtrl_SetClosedLoop(float leftSpeed, float rightSpeed, float posOffset = 0.0f);
// void WheelSpeedCtrl_SetOpenLoop(float leftSpeed, float rightSpeed);

// // 级联（速度外环+速度内环）初始化：会自动删除已有单环任务并创建新任务
// void WheelSpeedCascade_Init(void);

// #ifdef __cplusplus
// }
// #endif

// #endif // WHEEL_SPEED_CTRL_H
