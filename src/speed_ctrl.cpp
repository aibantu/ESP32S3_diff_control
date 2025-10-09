#include "speed_ctrl.h"

// 任务周期
static const TickType_t dt = pdMS_TO_TICKS(4); // 4ms

// ============= 内部状态 =============
TaskHandle_t speedTaskHandle = nullptr;
static volatile float left_speed = 0.0f;  // rad/s
static volatile float right_speed = 0.0f;  // rad/s
bool is_speedTask_created = false;

// ============= 配置参数（可按需调参） =================
LowPassFilter velFilter = LowPassFilter(0.001); // Tf = 1ms   //M0速度环
PIDController speedPid{2,0.05,0,10000,100};
float speed_limit_error = 0.5f;
//重新设置速度PID
void setSpeedPID(float P,float I,float D,float ramp,float limit)   //M0速度环PID设置
{
  speedPid.P=P;
  speedPid.I=I;
  speedPid.D=D;
  speedPid.output_ramp=ramp;
  speedPid.limit=limit;
}

//M0速度PID接口
float getSpeedPID(float error)
{
  return speedPid(error);
}

// 停止输出
void stopWheel(){
	Motor_SetTorque(&leftWheel, 0.0f);
	Motor_SetTorque(&rightWheel, 0.0f);
}

//有滤波
float getLeftVelocity()
{
  //获取速度数据并滤波
  float oriLeftVel=leftWheel.speed;
  float vel=velFilter(oriLeftVel);
  return vel;   //考虑方向
}

float getRightVelocity()
{
  //获取速度数据并滤波
  float oriRightVel=rightWheel.speed;
  float vel=velFilter(oriRightVel);
  return vel;   //考虑方向
}


void SpeedCtrl_SetTargets(float leftSpeed, float rightSpeed){
	left_speed = leftSpeed;
	right_speed = rightSpeed;
}

static void SpeedCtrl_Task(void *arg){
	TickType_t lastWake = xTaskGetTickCount();
	while(true){

		// 当前误差
		float errL = left_speed  - getLeftVelocity();
		float errR = right_speed - getRightVelocity();

    if (fabs(errL) < speed_limit_error){
      errL = 0.0;
    }
    if (fabs(errR) < speed_limit_error) {
      errR = 0.0;
    }

		// PID 输出（力矩指令)。右轮方向保持软件反转。
		float outL = getSpeedPID(errL);
		float outR = getSpeedPID(errR);
		// 输出给电机（右轮取反匹配既有差动约定）
		Motor_SetTorque(&leftWheel, outL);
		Motor_SetTorque(&rightWheel, outR);
		vTaskDelayUntil(&lastWake, dt);
	}
}

void SpeedCtrl_Init(){
    // 第一层检查：如果任务已创建，直接返回
    if (is_speedTask_created) {
        Serial.println("[SpeedCtrl] Task already created");
        return;
    }

    // 第二层检查：确保句柄也为空（双重保险）
    if (speedTaskHandle != nullptr) {
        Serial.println("[SpeedCtrl] Task handle exists");
        return;
    }
  // 创建任务
  BaseType_t result = xTaskCreate(SpeedCtrl_Task, "SpeedCtrl", 3072, nullptr, 4, &speedTaskHandle);
      // 只有创建成功，才标记任务已创建
  if (result == pdPASS) {
      is_speedTask_created = true;  // 标志位置为true，防止再次创建
      Serial.println("[SpeedCtrl] Task created successfully");
  } else {
      Serial.println("[SpeedCtrl] Failed to create task");
  }
}

