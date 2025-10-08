#include <Arduino.h>
#include "serial.h"
#include "can.h"
#include "imu.h"
#include "motor.h"
#include "ble.h"
#include "ctrl.h"
#include "adc.h"
#include "wheel_speed_ctrl.h"
void setup()
{
//优先级1
Serial_Init();
//优先级4
CAN_Init();
IMU_Init();
Motor_InitAll();
//状态更新优先级3    姿态控制优先级为1 
// Ctrl_Init();    //注释掉电机不转了
// WheelSpeedCtrl_Init();
// WheelSpeedCascade_Init();
WheelSpeedCtrl_Init();
//蓝牙遥控器捏
BLE_Init();
}

void loop()
{
}