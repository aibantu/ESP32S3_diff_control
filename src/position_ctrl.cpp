// #include "position_ctrl.h"

// // 任务周期
// static const TickType_t dt = pdMS_TO_TICKS(4); // 4ms

// // ============= 内部状态 =============
// TaskHandle_t posTaskHandle = nullptr;
// static volatile float left_position = 0.0f;
// static volatile float right_position = 0.0f;
// volatile bool is_posTask_created = false;

// // ============= 配置参数（可按需调参） =================
// // PID 构造: {Kp, Ki, Kd, ramp(输出变化率限制), limit(输出幅值限制)}
// // 旧参数: 1.5,0.1,0.0002,10,30  存在问题:
// // 1) limit=30 远超电机可用范围(后续会被电压限幅->长时间饱和, 积分累积) 导致过冲+振荡
// // 2) Ki 偏大 + 大步阶 (例如 60) -> 快速积分 windup
// // 3) Kd 极小, 几乎无阻尼
// // 新策略:
// // - 先关闭 ramp(设0) 便于观察真实响应, 如需柔化再开启
// // - 大幅降低 Ki, 并在接近目标时再逐步调高
// // - 提高 Kd 形成相位提前(阻尼), 抑制超调
// // - 限制输出幅值到接近实际可用扭矩范围 (<=  maxVoltage * torqueRatio ≈ 7 * 0.8 ≈ 5.6)
// //   给一点裕度取 6
// // 起始参数建议: Kp 0.9, Ki 0.02, Kd 0.05, ramp 0, limit 6
// PID leftAnglePid{0.9f,0.02f,0.05f,0.0f,6.0f};
// PID rightAnglePid{0.9f,0.02f,0.05f,0.0f,6.0f};
// // 位置误差死区 (度). 如果角度单位后续换成弧度请同步调整
// float position_limit_error = 0.05f; // TODO: 若噪声>0.05 可适当增大到 0.1


// void PositionCtrl_SetTargets(float leftAngle, float rightAngle, bool relative){
// 	if(relative){
// 		left_position  = leftWheel.current_angle  + leftAngle;
// 		right_position = rightWheel.current_angle + rightAngle;
// 	} else {
// 		left_position  = leftAngle;
// 		right_position = rightAngle;
// 	}
// }

// static void PositionCtrl_Task(void *arg){
// 	TickType_t lastWake = xTaskGetTickCount();
// 	while(true){

// 		// 当前误差
// 		float errL = left_position  - leftWheel.current_angle;
// 		float errR = right_position - rightWheel.current_angle;
//     if (fabs(errL) < position_limit_error){
//       errL = 0.0;
//     }
//     if (fabs(errR) < position_limit_error) {
//       errR = 0.0;
//     }

// 		// PID 输出（力矩指令)。右轮方向保持软件反转。
// 		// float outL = getAnglePID(errL);
// 		// float outR = getAnglePID(errR);
// 		float outL = getPID(&leftAnglePid, errL);
// 		float outR = getPID(&rightAnglePid, errR);
//     // anglePid.getPID(errL);
// 		// 输出给电机（右轮取反匹配既有差动约定）
// 		Motor_SetTorque(&leftWheel, outL);
// 		Motor_SetTorque(&rightWheel, outR);
// 		vTaskDelayUntil(&lastWake, dt);
// 	}
// }

// bool PositionCtrl_Active() { return posTaskHandle != nullptr; }

// void PositionCtrl_Stop() {
//     if (posTaskHandle) {
//         TaskHandle_t h = posTaskHandle;
//         posTaskHandle = nullptr;   // 先清空，防止竞态
//         is_posTask_created = false;
//         vTaskDelete(h);
//         vTaskDelay(60);
//         Serial.println("[PosCtrl] stopped");
//     }
// }

// void PositionCtrl_Init(){
//     // 第一层检查：如果任务已创建，直接返回
//     if (is_posTask_created) {
//         Serial.println("[PosCtrl] Task already created");
//         return;
//     }

//     // 第二层检查：确保句柄也为空（双重保险）
//     if (posTaskHandle != nullptr) {
//         Serial.println("[PosCtrl] Task handle exists");
//         return;
//     }
//   // 创建任务
//   BaseType_t result = xTaskCreate(PositionCtrl_Task, "PosCtrl", 3072, nullptr, 4, &posTaskHandle);
//       // 只有创建成功，才标记任务已创建
//   if (result == pdPASS) {
//       is_posTask_created = true;  // 标志位置为true，防止再次创建
//       Serial.println("[PosCtrl] Task created successfully");
//   } else {
//       Serial.println("[PosCtrl] Failed to create task");
//   }
// }

