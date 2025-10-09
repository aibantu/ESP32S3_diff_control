#include <Arduino.h>
#include "can.h"
#include "imu.h"
#include "motor.h"
#include "adc.h"
#include "position_ctrl.h"
#include "speed_ctrl.h"
void setup()
{
       Serial.begin(115200);
    // 等待主机打开串口（可选：最多等待2秒，防止卡死）
    unsigned long start = millis();
    while(!Serial && millis() - start < 2000) {
        delay(10);
    }
    //优先级1
    // Serial_Init();
    //优先级4
    CAN_Init();
    IMU_Init();
    Motor_InitAll();
    //状态更新优先级3    姿态控制优先级为1 
    // Ctrl_Init();    //注释掉电机不转了
    // WheelSpeedCtrl_Init();
    // WheelSpeedCascade_Init();
    // WheelSpeedCtrl_Init();
    // 原先这里调用 BLE_Init(); 但当前项目未使用 BLE，已移除引用。
}

void loop()
{
}