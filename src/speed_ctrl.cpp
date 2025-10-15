#include "speed_ctrl.h"

// 任务周期
static const TickType_t dt = pdMS_TO_TICKS(4); // 4ms

// ============= 内部状态 =============
TaskHandle_t speedTaskHandle = nullptr;
static volatile float left_speed = 0.0f;  // rad/s
static volatile float right_speed = 0.0f;  // rad/s
bool is_speedTask_created = false;

// ============= 配置参数（可按需调参） =================
LowPassFilter leftVelFilter = LowPassFilter(0.003); // Tf = 3ms   //M0速度环
LowPassFilter rightVelFilter = LowPassFilter(0.003); // Tf = 3ms   //M0速度环
// 优化后的速度环 PID 数值: {Kp, Ki, Kd, ramp, limit}
// 调参思路:
// - 原Kp=2.5偏高导致大步阶易超调；Ki=0.1 对电机+机械已含积分的系统偏大；Kd=0.01 阻尼不足。
// - ramp=10 限制输出变化率拖慢加速；limit=50 远超电机可实际利用的扭矩映射区间(易长时间饱和)。
// 新参数: Kp 1.30  提供足够响应
//        Ki 0.05  维持稳态消除误差又不过度积累
//        Kd 0.02  增强阻尼抑制超调
//        ramp 0   取消速率限制, 加快到达
//        limit 6  接近可用扭矩范围(约= maxVoltage*torqueRatio ≈ 7*0.8)
// 振动优化版本: 下调Kp与Ki, 提升Kd 形成更高阻尼
// 旧: 1.30 / 0.05 / 0.02  振动说明阻尼不足 + 积分稍高
// 新: 1.10 / 0.035 / 0.035  (若仍振动可继续: Kp 1.05, Ki 0.030, Kd 0.040)
PID leftSpeedPid{3.6f,0.35f,0.1f,30.0f,100.0f};
PID rightSpeedPid{3.6f,0.35f,0.1f,30.0f,100.0f};
float speed_limit_error = 0.5f;
//重新设置速度PID
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

    if (fabs(errL) < speed_limit_error){
      errL = 0.0;
    }
    if (fabs(errR) < speed_limit_error) {
      errR = 0.0;
    }

		// PID 输出（力矩指令)。右轮方向保持软件反转。
		float outL = getPID(&leftSpeedPid, errL);
		float outR = getPID(&rightSpeedPid, errR);
		// 输出给电机（右轮取反匹配既有差动约定）
		Motor_SetTorque(&leftWheel, outL);
		Motor_SetTorque(&rightWheel, outR);
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

