#ifndef SPEED_CTRL_H
#define SPEED_CTRL_H

#include <Arduino.h>
#include "pid.h"
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lowpass_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

void SpeedCtrl_Init();

// 设置目标速度（单位：rad/s）。
// relative = true 时按相对当前速度偏移；否则为绝对速度。
void SpeedCtrl_SetTargets(float leftSpeed, float rightSpeed);
// 停止输出
void stopWheel();

//有滤波
float getLeftVelocity();

float getRightVelocity();
bool SpeedCtrl_Active();
void SpeedCtrl_Stop();
static void SpeedCtrl_Task(void *arg);

#ifdef __cplusplus
}
#endif

#endif 

