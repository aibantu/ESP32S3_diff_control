// #include "wheel_speed_ctrl.h"
// #include <math.h>

// /*
//   轮速控制模块 - 简化稳定的双环控制
  
//   设计思路：
//   - 统一双环控制：位置外环 + 速度内环
//   - 速度模式：动态积分更新位置目标
//   - 位置模式：固定位置目标
//   - 彻底独立，不依赖ctrl.cpp的target变量
// */

// // 全局变量
// volatile uint32_t g_lastWheelCmdMs = 0;
// volatile float wheelSpeedRefL = 0.0f;
// volatile float wheelSpeedRefR = 0.0f;
// TaskHandle_t wheelSpeedTaskHandle = NULL;

// // 简化的控制状态
// static struct {
//     // PID控制器
//     CascadePID leftPID, rightPID;
    
//     // 目标值
//     float targetSpeedL, targetSpeedR;      // 目标速度 (rad/s)
//     float targetAngleL, targetAngleR;      // 目标位置 (rad)
    
//     // 控制模式
//     bool isPositionMode;                   // 位置模式标志
    
//     // 状态
//     bool isActive;                         // 控制激活标志
// } wheelCtrl = {0};

// #define CTRL_PERIOD_MS 4
// #define CMD_TIMEOUT_MS 5000

// // 简化稳定的控制任务
// void WheelSpeedTask(void *arg) {
//     TickType_t lastWakeTime = xTaskGetTickCount();
    
//     while (1) {
//         uint32_t now = millis();
        
//         // 超时保护
//         if ((now - g_lastWheelCmdMs) > CMD_TIMEOUT_MS) {
//             wheelCtrl.targetSpeedL = wheelCtrl.targetSpeedR = 0.0f;
//             wheelCtrl.isActive = false;
//         }
        
//         // 停机处理
//         if (!wheelCtrl.isActive) {
//             Motor_SetTorque(&leftWheel, 0);
//             Motor_SetTorque(&rightWheel, 0);
//             vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CTRL_PERIOD_MS));
//             continue;
//         }
        
//         // 速度模式：根据目标速度动态更新位置目标
//         if (!wheelCtrl.isPositionMode) {
//             // 积分更新位置目标，实现速度跟踪
//             wheelCtrl.targetAngleL += wheelCtrl.targetSpeedL * 0.004f;  // 4ms积分
//             wheelCtrl.targetAngleR += wheelCtrl.targetSpeedR * 0.004f;  // 右轮同向
            
//             // 限制位置偏移，防止积分发散
//             float maxDrift = 1.0f;  // 减小偏移限制
//             if (wheelCtrl.targetAngleL - leftWheel.angle > maxDrift) {
//                 wheelCtrl.targetAngleL = leftWheel.angle + maxDrift;
//             } else if (wheelCtrl.targetAngleL - leftWheel.angle < -maxDrift) {
//                 wheelCtrl.targetAngleL = leftWheel.angle - maxDrift;
//             }
            
//             if (wheelCtrl.targetAngleR - rightWheel.angle > maxDrift) {
//                 wheelCtrl.targetAngleR = rightWheel.angle + maxDrift;
//             } else if (wheelCtrl.targetAngleR - rightWheel.angle < -maxDrift) {
//                 wheelCtrl.targetAngleR = rightWheel.angle - maxDrift;
//             }
//         }
        
//         // 双环级联PID控制
//         PID_CascadeCalc(&wheelCtrl.leftPID, 
//                        wheelCtrl.targetAngleL,      // 目标位置
//                        leftWheel.angle,             // 实际位置
//                        leftWheel.speed);            // 实际速度
        
//         PID_CascadeCalc(&wheelCtrl.rightPID,
//                        wheelCtrl.targetAngleR,      // 目标位置  
//                        rightWheel.angle,            // 实际位置
//                        rightWheel.speed);           // 实际速度
        
//         // 输出扭矩 - 右轮需要反向以实现正确的差动
//         Motor_SetTorque(&leftWheel, wheelCtrl.leftPID.output);
//         Motor_SetTorque(&rightWheel, -wheelCtrl.rightPID.output);  // 右轮反向
        
//         vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CTRL_PERIOD_MS));
//     }
// }

// // 简化初始化函数
// void WheelSpeedCtrl_Init(void) {
//     // 外环(位置环) - 保守稳定的参数
//     PID_Init(&wheelCtrl.leftPID.outer, 0.4f, 0.01f, 0.005f, 0.8f, 10.0f);
//     PID_Init(&wheelCtrl.rightPID.outer, 0.4f, 0.01f, 0.005f, 0.8f, 10.0f);
    
//     // 内环(速度环) - 响应快但稳定的参数  
//     PID_Init(&wheelCtrl.leftPID.inner, 0.08f, 0.008f, 0.002f, 0.3f, 0.6f);
//     PID_Init(&wheelCtrl.rightPID.inner, 0.08f, 0.008f, 0.002f, 0.3f, 0.6f);
    
//     // 适当的死区设置
//     PID_SetDeadzone(&wheelCtrl.leftPID.outer, 0.02f);    
//     PID_SetDeadzone(&wheelCtrl.rightPID.outer, 0.02f);
//     PID_SetDeadzone(&wheelCtrl.leftPID.inner, 0.1f);     
//     PID_SetDeadzone(&wheelCtrl.rightPID.inner, 0.1f);
    
//     // 滤波设置  
//     PID_SetErrLpfRatio(&wheelCtrl.leftPID.outer, 0.9f);
//     PID_SetErrLpfRatio(&wheelCtrl.rightPID.outer, 0.9f);
//     PID_SetErrLpfRatio(&wheelCtrl.leftPID.inner, 0.7f);
//     PID_SetErrLpfRatio(&wheelCtrl.rightPID.inner, 0.7f);
    
//     // 初始化控制状态
//     wheelCtrl.targetAngleL = leftWheel.angle;
//     wheelCtrl.targetAngleR = rightWheel.angle;
//     wheelCtrl.targetSpeedL = 0.0f;
//     wheelCtrl.targetSpeedR = 0.0f;
//     wheelCtrl.isPositionMode = false;
//     wheelCtrl.isActive = false;
    
//     // 创建控制任务
//     if (wheelSpeedTaskHandle) {
//         vTaskDelete(wheelSpeedTaskHandle);
//     }
//     xTaskCreate(WheelSpeedTask, "WheelCtrl", 3072, NULL, 4, &wheelSpeedTaskHandle);
    
//     g_lastWheelCmdMs = millis();
//     Serial.println("[WheelCtrl] Initialized - Dual-loop cascade control");
// }

// // 简化的设置函数
// void WheelSpeedCtrl_SetClosedLoop(float leftSpeed, float rightSpeed, float posOffset) {
//     g_lastWheelCmdMs = millis();
    
//     // 激活控制
//     wheelCtrl.isActive = true;
    
//     // 位置模式
//     if (posOffset != 0.0f) {
//         wheelCtrl.isPositionMode = true;
//         wheelCtrl.targetAngleL = leftWheel.angle + posOffset;
//         wheelCtrl.targetAngleR = rightWheel.angle + posOffset;
        
//         Serial.print("Position: offset=");
//         Serial.println(posOffset, 2);
//     } 
//     // 速度模式
//     else {
//         wheelCtrl.isPositionMode = false;
//         wheelCtrl.targetSpeedL = leftSpeed;
//         wheelCtrl.targetSpeedR = rightSpeed;
        
//         // 初始位置设为当前位置，避免跳跃
//         wheelCtrl.targetAngleL = leftWheel.angle;
//         wheelCtrl.targetAngleR = rightWheel.angle;
        
//         Serial.print("Speed: L=");
//         Serial.print(leftSpeed, 1);
//         Serial.print(" R=");
//         Serial.println(rightSpeed, 1);
//     }
    
//     // 清除PID历史
//     PID_Clear(&wheelCtrl.leftPID.inner);
//     PID_Clear(&wheelCtrl.leftPID.outer);
//     PID_Clear(&wheelCtrl.rightPID.inner);
//     PID_Clear(&wheelCtrl.rightPID.outer);
    
//     // 兼容性
//     wheelSpeedRefL = leftSpeed;
//     wheelSpeedRefR = rightSpeed;
// }

// // 开环控制(简化版)
// void WheelSpeedCtrl_SetOpenLoop(float leftSpeed, float rightSpeed) {
//     wheelCtrl.isActive = false;  // 禁用闭环控制
    
//     float maxTorque = leftWheel.maxVoltage * leftWheel.torqueRatio * motorOutRatio;
//     float torqueL = leftSpeed * 0.08f;   // 开环映射系数
//     float torqueR = rightSpeed * 0.08f;
    
//     // 扭矩限幅
//     if (torqueL > maxTorque) torqueL = maxTorque;
//     else if (torqueL < -maxTorque) torqueL = -maxTorque;
//     if (torqueR > maxTorque) torqueR = maxTorque;
//     else if (torqueR < -maxTorque) torqueR = -maxTorque;
    
//     // 右轮反向
//     Motor_SetTorque(&leftWheel, torqueL);
//     Motor_SetTorque(&rightWheel, -torqueR);
    
//     g_lastWheelCmdMs = millis();
//     Serial.println("Open-loop mode");
// }
