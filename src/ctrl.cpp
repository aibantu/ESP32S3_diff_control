#include <Arduino.h>
#include "pid.h"
#include "motor.h"
#include "ctrl.h"
#include "imu.h"
#include "wheel_speed_ctrl.h"
#include <esp_task_wdt.h>
#include <math.h>

CascadePID yawPID; //机身yaw和roll控制PID


Target target = {0, 0, 0, 0, 0, 0, 0};
StateVar stateVar;

//目标量更新任务(根据蓝牙收到的目标量计算实际控制算法的给定量)
void Ctrl_TargetUpdateTask(void *arg)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	float speedSlopeStep = 0.003f;
	while (1)
	{
		//计算速度斜坡，斜坡值更新到target.speed
		if(fabs(target.speedCmd - target.speed) < speedSlopeStep)
			target.speed = target.speedCmd;
		else
		{
			if(target.speedCmd - target.speed > 0)
				target.speed += speedSlopeStep;
			else
				target.speed -= speedSlopeStep;
		}
		//计算位置目标，并限制在当前位置的±0.1m内
		target.position += target.speed * 0.004f;
		if(target.position - stateVar.x > 0.1f)
			target.position = stateVar.x + 0.1f; 
		else if(target.position - stateVar.x < -0.1f)
			target.position = stateVar.x - 0.1f;
		//限制速度目标在当前速度的±0.3m/s内
		if(target.speed - stateVar.dx > 0.3f)
			target.speed = stateVar.dx + 0.3f;
		else if(target.speed - stateVar.dx < -0.3f)
			target.speed = stateVar.dx - 0.3f;
		//计算yaw方位角目标
		target.yawAngle += target.yawSpeedCmd * 0.004f;
		vTaskDelayUntil(&xLastWakeTime, 4); //每4ms更新一次
	}
}



void CtrlBasic_Task(void *arg)
{
	const float wheelRadius = 0.04; //m，车轮半径
	TickType_t xLastWakeTime = xTaskGetTickCount();

	//设定初始目标值
	target.rollAngle = 0.0f;
	target.speed = 0.0f;
	target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
	bool increasing = true; // 角度是否在增加
	while (1)
	{
		// 一次性 yaw 对齐与 PID 清零：避免上电初始 yaw 偏差造成的瞬间大输出
		static bool yawAligned = false;
		if(!yawAligned) {
			// 直接将目标 yaw 置为当前测量值，并清空串级 PID 状态
			target.yawAngle = imuData.yaw;
			PID_Clear(&yawPID.inner);
			PID_Clear(&yawPID.outer);
			yawAligned = true;
			Serial.println("[Ctrl] Yaw aligned & PID cleared");
		}
		// 如果当前为车轮速度模式（闭环或开环），则跳过 yaw 扭矩写入，由 WheelSpeedTask 负责
		if (g_ctrlMode == CTRL_MODE_WHEEL_SPEED_CLOSED || g_ctrlMode == CTRL_MODE_WHEEL_SPEED_OPEN) {
			vTaskDelayUntil(&xLastWakeTime, 4);
			continue;
		}
		//计算状态变量
		stateVar.x = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
		stateVar.dx = (leftWheel.speed + rightWheel.speed) / 2 * wheelRadius;

		//准备状态变量
		float x[2] = {stateVar.x, stateVar.dx};
		//与给定量作差
		x[0] -= target.position;
		x[1] -= target.speed;

		//计算yaw轴PID输出
		PID_CascadeCalc(&yawPID, target.yawAngle, imuData.yaw, imuData.yawSpd);
		
		//设定车轮电机输出扭矩，为LQR和yaw轴PID输出的叠加
		if(target.rollAngle<10) //正常接地状态
		{
			Motor_SetTorque(&leftWheel, -yawPID.output);
			Motor_SetTorque(&rightWheel, yawPID.output);
		}
		else //侧翻状态，关闭车轮电机
		{
			Motor_SetTorque(&leftWheel, 0);
			Motor_SetTorque(&rightWheel, 0);
		}

		vTaskDelayUntil(&xLastWakeTime, 4); //4ms控制周期
	}
}

void Ctrl_Init(void)
{

	PID_Init(&yawPID.inner, 0.01, 0, 0, 0, 0.1);
	PID_Init(&yawPID.outer, 6, 0, 0, 0, 1);

	xTaskCreate(Ctrl_TargetUpdateTask, "Ctrl_TargetUpdateTask", 4096, NULL, 3, NULL);
	vTaskDelay(2);
	//xTaskCreate(Ctrl_StandupPrepareTask, "StandupPrepare_Task", 4096, NULL, 1, NULL);
	//xTaskCreate(vSinGeneratorTask, "vSinGeneratorTask", 4096, NULL, 1, NULL);
	//xTaskCreate(VMC_TestTask, "VMC_TestTask", 4096, NULL, 1, NULL);
	xTaskCreate(CtrlBasic_Task, "CtrlBasic_Task", 4096, NULL, 1, NULL);
	// 新增：初始化车轮速度控制模块
	WheelSpeedCtrl_Init();
}