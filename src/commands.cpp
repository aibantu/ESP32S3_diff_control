#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "commands.h"
#include "position_ctrl.h"
#include "speed_ctrl.h"
#include "motor.h"  // 为 g/h/j 查询电机数据

#define USB_CMD_TASK_STARTUP_DELAY_MS 1500
// ===== 状态机缓冲 =====
static int sm_arg = 0;      // 0=命令 1=参数1 2=参数2 3=参数3
static int sm_index = 0;    // 当前写入索引
static char sm_cmd = 0;     // 命令字符
static char sm_argv1[16];
static char sm_argv2[16];
static char sm_argv3[16];
static long sm_arg1 = 0;    // 解析后的数值参数1
static long sm_arg2 = 0;    // 解析后的数值参数2
static float sm_arg3 = 0;   // 解析后的数值参数3(浮点)
static inline void sm_reset(){
    sm_cmd = 0; sm_arg = 0; sm_index = 0; sm_arg1 = sm_arg2 = 0; sm_arg3 = 0;
    memset(sm_argv1, 0, sizeof(sm_argv1));
    memset(sm_argv2, 0, sizeof(sm_argv2));
    memset(sm_argv3, 0, sizeof(sm_argv3));
}

// 命令执行
static void sm_run(){
    sm_arg1 = atol(sm_argv1);
    sm_arg2 = atol(sm_argv2);
    sm_arg3 = atof(sm_argv3);
    switch (sm_cmd){
        case GET_BAUDRATE: // 'b'
            Serial.println("57600");
            break;
        case ANALOG_READ: // 'a'
            Serial.println(analogRead((int)sm_arg1));
            break;
        case DIGITAL_READ: // 'd'
            Serial.println(digitalRead((int)sm_arg1));
            break;
        case PIN_MODE: {   // 'c' : c <pin> <mode>
            bool ok = true;
            switch (sm_arg2){
                case 0: pinMode((int)sm_arg1, INPUT); break;
                case 1: pinMode((int)sm_arg1, OUTPUT); break;
            }
            Serial.println(ok?"OK":"ERR");
            break; }
        case DIGITAL_WRITE: // 'w' : w <pin> <val>
            digitalWrite((int)sm_arg1, sm_arg2?HIGH:LOW);
            Serial.println("OK");
            break;
        case ANALOG_WRITE: { // 'x' : x <pin> <duty>
            int duty = (int)sm_arg2;
            if (duty < 0) duty = 0; else if (duty > 255) duty = 255;
            analogWrite((int)sm_arg1, duty);
            Serial.print("ANALOGWRITE "); Serial.print((int)sm_arg1); Serial.print(' '); Serial.println(duty);
            break; }
        case 'm': { // 闭环速度控制：m <left_rad_s> <right_rad_s> [position_offset]
            float leftSpeed = (float)sm_arg1;
            float rightSpeed = (float)sm_arg2;
            float posOffset = sm_arg3; // 第三个参数为位置偏移(可选)
            // WheelSpeedCtrl_SetClosedLoop(leftSpeed, rightSpeed, posOffset);
            Serial.println("OK");
            break; }
        case MOTOR_ANGLE_AND_SPEED: { // 'j' 返回：leftAngle rightAngle leftSpeed rightSpeed（空格分隔）
            Serial.print(leftWheel.current_angle, 3);
            Serial.print(' ');
            Serial.print(rightWheel.current_angle, 3);
            Serial.print(' ');
            Serial.print(leftWheel.speed, 3);
            Serial.print(' ');
            Serial.println(rightWheel.speed, 3);
            break; }
        case 'p': { // 'p' : p <left_angle> <right_angle> [r]
            // 参数：leftAngle(rad) rightAngle(rad) 可选 第三参数=字符r表示相对模式，否则绝对
            float leftAngle = atof(sm_argv1);
            float rightAngle = atof(sm_argv2);
            bool relative = false;
            // 仅当第三参数存在且为 'r' 时才按相对模式
            if (sm_argv3[0] != 0 && sm_argv3[0] == 'r') relative = true;
            if (SpeedCtrl_Active()) {
                SpeedCtrl_Stop();
            }
            PositionCtrl_Init();
            PositionCtrl_SetTargets(leftAngle, rightAngle, relative);
            Serial.print("Pos Target L:"); Serial.print(leftAngle, 3);
            Serial.print(" R:"); Serial.print(rightAngle, 3);
            Serial.print(relative?" (rel)":" (abs)");
            Serial.println();
            break; }
        case 's': { // 's' : s <left_rad_s> <right_rad_s>
            float leftSpeed = atof(sm_argv1);
            float rightSpeed = atof(sm_argv2);
            if (PositionCtrl_Active()) {
                PositionCtrl_Stop();
            }
            SpeedCtrl_Init();
            SpeedCtrl_SetTargets(leftSpeed, rightSpeed);
            Serial.print("Speed Target L:"); Serial.print(leftSpeed, 3);
            Serial.print(" R:"); Serial.print(rightSpeed, 3);
            Serial.println();
            break; }
        default:
            Serial.println("Invalid Command");
            break;
    }
}

static void USB_CommandTask(void *pvParameters){
    vTaskDelay(pdMS_TO_TICKS(USB_CMD_TASK_STARTUP_DELAY_MS));
    Serial.println("USB cmd state machine started");
    sm_reset();
    while (1){
        while (Serial.available() > 0){
            char ch = Serial.read();
            if (ch == '\r' || ch == '\n'){
                if (sm_arg == 1) sm_argv1[sm_index] = 0; 
                else if (sm_arg == 2) sm_argv2[sm_index] = 0;
                else if (sm_arg == 3) sm_argv3[sm_index] = 0;
                if (sm_cmd) sm_run();
                sm_reset();
                continue;
            }
            if (ch == ' ' || ch == '\t'){
                if (sm_arg == 0) { sm_arg = 1; sm_index = 0; }
                else if (sm_arg == 1){ sm_argv1[sm_index] = 0; sm_arg = 2; sm_index = 0; }
                else if (sm_arg == 2){ sm_argv2[sm_index] = 0; sm_arg = 3; sm_index = 0; }
                continue;
            }
            if (sm_arg == 0){
                // 统一大小写：命令字符转为小写
                if(ch >= 'A' && ch <= 'Z') ch = ch - 'A' + 'a';
                sm_cmd = ch;
            } else if (sm_arg == 1){
                if (sm_index < (int)sizeof(sm_argv1)-1) sm_argv1[sm_index++] = ch;
            } else if (sm_arg == 2){
                if (sm_index < (int)sizeof(sm_argv2)-1) sm_argv2[sm_index++] = ch;
            } else if (sm_arg == 3){
                if (sm_index < (int)sizeof(sm_argv3)-1) sm_argv3[sm_index++] = ch;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 使用构造函数属性在程序启动早期创建任务（不修改 main.cpp 的前提下启动）
__attribute__((constructor)) static void create_usb_command_task(void){
    xTaskCreate(USB_CommandTask, "USB_CmdTask", 4096, NULL, 1, NULL);  // 增加栈大小
}
