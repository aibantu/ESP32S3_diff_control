// // position_ctrl.h  基于电机角度的简易双轮位置闭环控制
// // 通过串口命令 'P <left_angle> <right_angle> [R]' 进入位置控制
// // 若第三参数为字母 R/r 表示相对模式(在当前角度基础上加偏移)，否则视为绝对目标
// // 与速度控制(m命令)互斥：进入位置模式会自动停用速度闭环(若存在)

// #ifndef POSITION_CTRL_H
// #define POSITION_CTRL_H

// #include <Arduino.h>
// #include "pid.h"
// #include "motor.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "lowpass_filter.h"
// #ifdef __cplusplus
// extern "C" {
// #endif

// // 初始化（幂等）。创建位置控制任务，但默认未激活。
// void PositionCtrl_Init();
// // 设置目标角度（单位：rad）。
// // relative = true 时按相对当前角度偏移；否则为绝对角度。
// void PositionCtrl_SetTargets(float leftAngle, float rightAngle, bool relative);
// void setAnglePID(float P,float I,float D,float ramp,float limit);
// //M0角度PID接口
// float getAnglePID(float error);
// static void PositionCtrl_Task(void *arg);
// bool PositionCtrl_Active();
// void PositionCtrl_Stop();

// #ifdef __cplusplus
// }
// #endif

// #endif // POSITION_CTRL_H

