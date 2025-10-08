// position_ctrl.cpp
// 简易位置闭环：单层角度 PID 输出扭矩（不再做速度内环，保持简单）。
// 适合测试：定位 / 相对位移。

#include "position_ctrl.h"
#include "pid.h"
#include "motor.h"
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
// ============= 配置参数（可按需调参） =================
// 收敛判定阈值
static float position_limit_error = 0.1f;      // 误差阈值 rad
static float Kp = 1.3f;
// 任务周期
static const TickType_t dt = pdMS_TO_TICKS(4); // 4ms

// ============= 内部状态 =============
static TaskHandle_t is_posTaskHandle = nullptr;
// static bool position_active = false;
static volatile float left_position = 0.0f;
static volatile float right_position = 0.0f;
static bool is_postask_created = false;

// 停止输出
void PositionCtrl_Disable(){
	Motor_SetTorque(&leftWheel, 0.0f);
	Motor_SetTorque(&rightWheel, 0.0f);
}

void PositionCtrl_SetTargets(float leftAngle, float rightAngle, bool relative){
	if(relative){
		left_position  = leftWheel.angle  + leftAngle;
		right_position = rightWheel.angle + rightAngle;
	} else {
		left_position  = leftAngle;
		right_position = rightAngle;
	}
  // position_active = true;
}

static void PositionCtrl_Task(void *arg){
	TickType_t lastWake = xTaskGetTickCount();
	while(true){

		// 当前误差
		float errL = left_position  - leftWheel.angle;
		float errR = right_position - rightWheel.angle;

    if (fabs(errL) < position_limit_error){
      errL = 0.0;
    }
    if (fabs(errR) < position_limit_error) {
      errR = 0.0;
    }

		// PID 输出（力矩指令)。右轮方向保持软件反转。
		float outL = Kp*errL;
		float outR = Kp*errR;
    _constrain(outL,-7,9);
    _constrain(outR,-7,9);
		// 输出给电机（右轮取反匹配既有差动约定）
		Motor_SetTorque(&leftWheel, outL);
		Motor_SetTorque(&rightWheel, outR);
		vTaskDelayUntil(&lastWake, dt);
	}
}

void PositionCtrl_Init(){
    // 第一层检查：如果任务已创建，直接返回
    if (is_postask_created) {
        Serial.println("[PosCtrl] Task already created");
        return;
    }

    // 第二层检查：确保句柄也为空（双重保险）
    if (is_posTaskHandle != nullptr) {
        Serial.println("[PosCtrl] Task handle exists");
        return;
    }
  // 创建任务
  BaseType_t result = xTaskCreate(PositionCtrl_Task, "PosCtrl", 3072, nullptr, 4, &is_posTaskHandle);
      // 只有创建成功，才标记任务已创建
  if (result == pdPASS) {
      is_postask_created = true;  // 标志位置为true，防止再次创建
      Serial.println("[PosCtrl] Task created successfully");
  } else {
      Serial.println("[PosCtrl] Failed to create task");
  }
}

