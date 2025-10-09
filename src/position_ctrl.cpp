#include "position_ctrl.h"

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
// ============= 配置参数（可按需调参） =================
PIDController anglePid{2,0.05,0,10000,100};
float position_limit_error = 0.05f;
//重新设置角度PID
void setAnglePID(float P,float I,float D,float ramp,float limit)   //M0角度环PID设置
{
  anglePid.P=P;
  anglePid.I=I;
  anglePid.D=D;
  anglePid.output_ramp=ramp;
  anglePid.limit=limit;
}

//M0角度PID接口
float getAnglePID(float error)
{
  return anglePid(error);
}


// 任务周期
static const TickType_t dt = pdMS_TO_TICKS(4); // 4ms

// ============= 内部状态 =============
TaskHandle_t posTaskHandle = nullptr;
static volatile float left_position = 0.0f;
static volatile float right_position = 0.0f;
volatile bool is_posTask_created = false;

void PositionCtrl_SetTargets(float leftAngle, float rightAngle, bool relative){
	if(relative){
		left_position  = leftWheel.current_angle  + leftAngle;
		right_position = rightWheel.current_angle + rightAngle;
	} else {
		left_position  = leftAngle;
		right_position = rightAngle;
	}
}

static void PositionCtrl_Task(void *arg){
	TickType_t lastWake = xTaskGetTickCount();
	while(true){

		// 当前误差
		float errL = left_position  - leftWheel.current_angle;
		float errR = right_position - rightWheel.current_angle;

    if (fabs(errL) < position_limit_error){
      errL = 0.0;
    }
    if (fabs(errR) < position_limit_error) {
      errR = 0.0;
    }

		// PID 输出（力矩指令)。右轮方向保持软件反转。
		float outL = getAnglePID(errL);
		float outR = getAnglePID(errR);
    _constrain(outL,-9,9);
    _constrain(outR,-9,9);
		// 输出给电机（右轮取反匹配既有差动约定）
		Motor_SetTorque(&leftWheel, outL);
		Motor_SetTorque(&rightWheel, outR);
		vTaskDelayUntil(&lastWake, dt);
	}
}

void PositionCtrl_Init(){
    // 第一层检查：如果任务已创建，直接返回
    if (is_posTask_created) {
        Serial.println("[PosCtrl] Task already created");
        return;
    }

    // 第二层检查：确保句柄也为空（双重保险）
    if (posTaskHandle != nullptr) {
        Serial.println("[PosCtrl] Task handle exists");
        return;
    }
  // 创建任务
  BaseType_t result = xTaskCreate(PositionCtrl_Task, "PosCtrl", 3072, nullptr, 4, &posTaskHandle);
      // 只有创建成功，才标记任务已创建
  if (result == pdPASS) {
      is_posTask_created = true;  // 标志位置为true，防止再次创建
      Serial.println("[PosCtrl] Task created successfully");
  } else {
      Serial.println("[PosCtrl] Failed to create task");
  }
}

