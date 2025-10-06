/*
    commands.cpp
    精简命令解析（参考提供的 command_processor 状态机思路），在 USB Serial 上实现：
        b                 -> 打印当前波特率 115200
        a <pin>           -> 模拟读取 (analogRead)
        d <pin>           -> 数字读取 (digitalRead)
        c <pin> <mode>    -> 设置 pinMode，mode: 0=INPUT 1=OUTPUT 2=INPUT_PULLUP 3=INPUT_PULLDOWN(若支持)
        w <pin> <val>     -> digitalWrite(val!=0?HIGH:LOW)
        x <pin> <duty>    -> analogWrite，占空 0-255

    解析规则：
        - 按字符流解析，空格分隔参数，以 \r 或 \n 结束命令；忽略多余空格。
        - 最多两个参数（argv1, argv2），不足时缺省为 0。
        - 与参考实现一致：状态机变量 arg(0/1/2) 管理当前写入的目标。

    说明：
        - 仅保留基础硬件交互命令，未加入差速/电机/编码器等扩展逻辑，便于最小化维护。
        - 可后续无缝扩展：在 switch(cmd) 中添加 case。
*/

// ===== 依赖与宏 =====
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "../include/commands.h"
#include "wheel_speed_ctrl.h"
#include "motor.h"  // 为 g/h/j 查询电机数据

#define USB_CMD_TASK_STARTUP_DELAY_MS 1500

// ===== 状态机缓冲 =====
static int sm_arg = 0;      // 0=命令 1=参数1 2=参数2
static int sm_index = 0;    // 当前写入索引
static char sm_cmd = 0;     // 命令字符
static char sm_argv1[16];
static char sm_argv2[16];
static long sm_arg1 = 0;    // 解析后的数值参数1
static long sm_arg2 = 0;    // 解析后的数值参数2

static inline void sm_reset(){
    sm_cmd = 0; sm_arg = 0; sm_index = 0; sm_arg1 = sm_arg2 = 0;
    memset(sm_argv1, 0, sizeof(sm_argv1));
    memset(sm_argv2, 0, sizeof(sm_argv2));
}

// 命令执行
static void sm_run(){
    sm_arg1 = atol(sm_argv1);
    sm_arg2 = atol(sm_argv2);
    switch (sm_cmd){
        case GET_BAUDRATE: // 'b'
            Serial.println("115200");
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
#ifdef INPUT_PULLUP
                case 2: pinMode((int)sm_arg1, INPUT_PULLUP); break;
#else
                case 2: ok=false; break;
#endif
#ifdef INPUT_PULLDOWN
                case 3: pinMode((int)sm_arg1, INPUT_PULLDOWN); break;
#else
                case 3: ok=false; break;
#endif
                default: ok=false; break;
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
        case 'm': { // 闭环速度：m <left_rad_s> <right_rad_s>
            float l = (float)sm_arg1;
            float r = (float)sm_arg2;
            WheelSpeedCtrl_SetClosedLoop(l, r);
            Serial.println("OK");
            break; }
        case 'o': { // 开环速度：o <left_rad_s> <right_rad_s>
            float l = (float)sm_arg1;
            float r = (float)sm_arg2;
            WheelSpeedCtrl_SetOpenLoop(l, r);
            Serial.println("OK");
            break; }
        case MOTOR_RAW_ANGLE: { // 'g' 返回左右轮原始角度 rawAngle
            float l = leftWheel.rawAngle;
            float r = rightWheel.rawAngle;
            Serial.printf("%.3f %.3f\n", l, r);
            break; }
        case MOTOR_ANGLE: { // 'h' 返回左右轮校正后角度 angle
            float l = leftWheel.angle;
            float r = rightWheel.angle;
            Serial.printf("%.3f %.3f\n", l, r);
            break; }
        case MOTOR_SPEED: { // 'j' 返回左右轮速度 speed(rad/s)
            float l = leftWheel.speed;
            float r = rightWheel.speed;
            Serial.printf("%.3f %.3f\n", l, r);
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
                if (sm_arg == 1) sm_argv1[sm_index] = 0; else if (sm_arg == 2) sm_argv2[sm_index] = 0;
                if (sm_cmd) sm_run();
                sm_reset();
                continue;
            }
            if (ch == ' ' || ch == '\t'){
                if (sm_arg == 0) { sm_arg = 1; sm_index = 0; }
                else if (sm_arg == 1){ sm_argv1[sm_index] = 0; sm_arg = 2; sm_index = 0; }
                continue;
            }
            if (sm_arg == 0){
                sm_cmd = ch;
            } else if (sm_arg == 1){
                if (sm_index < (int)sizeof(sm_argv1)-1) sm_argv1[sm_index++] = ch;
            } else if (sm_arg == 2){
                if (sm_index < (int)sizeof(sm_argv2)-1) sm_argv2[sm_index++] = ch;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 使用构造函数属性在程序启动早期创建任务（不修改 main.cpp 的前提下启动）
__attribute__((constructor)) static void create_usb_command_task(void){
    xTaskCreate(USB_CommandTask, "USB_CmdTask", 2048, NULL, 1, NULL);
}
