#ifndef WHEEL_SPEED_CTRL_H
#define WHEEL_SPEED_CTRL_H
#include <Arduino.h>
#include "pid.h"
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CTRL_MODE_YAW = 0,
    CTRL_MODE_WHEEL_SPEED_CLOSED,
    CTRL_MODE_WHEEL_SPEED_OPEN
} ControlMode;

extern volatile ControlMode g_ctrlMode;
extern volatile uint32_t g_lastWheelCmdMs;
extern volatile float wheelSpeedRefL;
extern volatile float wheelSpeedRefR;

// 任务句柄（可被外部用于删除/判断）
extern TaskHandle_t wheelSpeedTaskHandle;

// 任务函数（允许在外部重新创建）
void WheelSpeedTask(void *arg);

void WheelSpeedCtrl_Init(void);
void WheelSpeedCtrl_SetClosedLoop(float leftRad, float rightRad);
void WheelSpeedCtrl_SetOpenLoop(float leftRad, float rightRad);

#ifdef __cplusplus
}
#endif

#endif // WHEEL_SPEED_CTRL_H
