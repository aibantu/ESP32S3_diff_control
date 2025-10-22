#include "speed_ctrl.h"

// 任务周期
static const TickType_t dt = pdMS_TO_TICKS(4); // 4ms

// ============= 内部状态 =============
TaskHandle_t speedTaskHandle = nullptr;
static volatile float left_speed = 0.0f;  // rad/s
static volatile float right_speed = 0.0f;  // rad/s
bool is_speedTask_created = false;

// ============= 配置参数（可按需调参） =================
LowPassFilter leftVelFilter = LowPassFilter(0.005); // Tf = 3ms   //M0速度环
LowPassFilter rightVelFilter = LowPassFilter(0.005); // Tf = 3ms   //M0速度环
// 优化后的速度环 PID 数值: {Kp, Ki, Kd, ramp, limit}

PID leftSpeedPid{3.0f,0.22f,0.06f,30.0f,100.0f};
PID rightSpeedPid{3.0f,0.22f,0.06f,30.0f,100.0f};
const float zero_speed_threshold = 0.12f; // rad/s, 可调
static const float torque_dead_zone = 0.06f;        // 扭矩输出死区（N·m或等效单位）
//重新设置速度PID
// 停止输出

void stopLeftWheel(){
  leftSpeedPid.integral_prev = 0.0f;
  leftSpeedPid.output_prev = 0.0f;
  leftSpeedPid.error_prev = 0.0f;
  leftSpeedPid.timestamp_prev = micros();
	Motor_SetTorque(&leftWheel, 0.0f);
}

void stopRightWheel(){
  rightSpeedPid.integral_prev = 0.0f;
  rightSpeedPid.output_prev = 0.0f;
  rightSpeedPid.error_prev = 0.0f;
  rightSpeedPid.timestamp_prev = micros();
	Motor_SetTorque(&rightWheel, 0.0f);
}

void stopWheel(){
    stopLeftWheel();
    stopRightWheel();
}

//有滤波
float getLeftVelocity()
{
  //获取速度数据并滤波
  float oriLeftVel=leftWheel.speed;
  float vel=leftVelFilter(oriLeftVel);
  return vel;   //考虑方向
}

float getRightVelocity()
{
  //获取速度数据并滤波
  float oriRightVel=rightWheel.speed;
  float vel=rightVelFilter(oriRightVel);
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

    if (fabs(errL) < zero_speed_threshold){
      errL = 0.0;
    }
    if (fabs(errR) < zero_speed_threshold) {
      errR = 0.0;
    }


    if (left_speed == 0.0f) {
        if (fabs(getLeftVelocity()) < zero_speed_threshold) {
            stopLeftWheel(); // 速度足够低时清零
        } else {
            // 速度未达标时，保留PID调节但降低积分增益（避免过度积累）
            float temp_I = leftSpeedPid.I;
            leftSpeedPid.I *= 0.3f; // 临时降低积分强度
            float outL = getPID(&leftSpeedPid, errL);
            if (fabs(outL) < torque_dead_zone) outL = 0.0f;
            Motor_SetTorque(&leftWheel, outL);
            leftSpeedPid.I = temp_I; // 恢复原积分增益
        }
    } else {
        float outL = getPID(&leftSpeedPid, errL);
        if (fabs(outL) < torque_dead_zone) outL = 0.0f;
        Motor_SetTorque(&leftWheel, outL);
    }

    if (right_speed == 0.0f) {
        if (fabs(getRightVelocity()) < zero_speed_threshold) {
            stopRightWheel(); // 速度足够低时清零
        } else {
            // 速度未达标时，保留PID调节但降低积分增益（避免过度积累）
            float temp_I = rightSpeedPid.I;
            rightSpeedPid.I *= 0.3f; // 临时降低积分强度
            float outR = getPID(&rightSpeedPid, errR);
            if (fabs(outR) < torque_dead_zone) outR = 0.0f;
            Motor_SetTorque(&rightWheel, outR);
            rightSpeedPid.I = temp_I; // 恢复原积分增益
        }
    } else {
        float outR = getPID(&rightSpeedPid, errR);
        if (fabs(outR) < torque_dead_zone) outR = 0.0f;
        Motor_SetTorque(&rightWheel, outR);
    }

		vTaskDelayUntil(&lastWake, dt);
	}
}

bool SpeedCtrl_Active() { return speedTaskHandle != nullptr; }

void SpeedCtrl_Stop() {
    if (speedTaskHandle) {
        TaskHandle_t h = speedTaskHandle;
        speedTaskHandle = nullptr;   // 先清空，防止竞态
        is_speedTask_created = false;
        vTaskDelete(h);
        vTaskDelay(60);
        stopWheel(); // 可选：保证电机停止
        Serial.println("[SpeedCtrl] stopped");
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

