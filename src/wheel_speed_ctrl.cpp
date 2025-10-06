#include "wheel_speed_ctrl.h"
#include <math.h>

// 任务句柄（对外提供）
TaskHandle_t wheelSpeedTaskHandle = NULL;
// 全局控制模式与参考
volatile ControlMode g_ctrlMode = CTRL_MODE_YAW;
volatile uint32_t g_lastWheelCmdMs = 0;
volatile float wheelSpeedRefL = 0.0f;
volatile float wheelSpeedRefR = 0.0f;

static PID wheelPidL, wheelPidR;

// 配置常量
#define WHEEL_PID_KP 0.9f
#define WHEEL_PID_KI 0.0f
#define WHEEL_PID_KD 0.03f
#define WHEEL_PID_MAX_SUM 0.5f
#define WHEEL_PID_MAX_OUT 0.9f

#define WHEEL_CTRL_PERIOD_MS 8
#define WHEEL_CMD_TIMEOUT_MS 900
#define MAX_WHEEL_SPEED_RAD 25.0f     // 速度参考限幅
#define REF_SLEW_STEP (2.0f)          // 每周期最大参考增量(rad/s)（可调）

// 开环参数
#define OPEN_K_LINEAR 0.03f
#define OPEN_K_BIAS   0.08f

static float slewLimit(float target, float current, float step){
    float d = target - current;
    if (d > step) d = step; else if (d < -step) d = -step;
    return current + d;
}

void WheelSpeedCtrl_SetClosedLoop(float leftRad, float rightRad){
    if (fabsf(leftRad) > MAX_WHEEL_SPEED_RAD) leftRad = (leftRad>0?MAX_WHEEL_SPEED_RAD:-MAX_WHEEL_SPEED_RAD);
    if (fabsf(rightRad) > MAX_WHEEL_SPEED_RAD) rightRad = (rightRad>0?MAX_WHEEL_SPEED_RAD:-MAX_WHEEL_SPEED_RAD);
    wheelSpeedRefL = leftRad;
    wheelSpeedRefR = rightRad;
    g_lastWheelCmdMs = millis();
    g_ctrlMode = CTRL_MODE_WHEEL_SPEED_CLOSED;
}

void WheelSpeedCtrl_SetOpenLoop(float leftRad, float rightRad){
    if (fabsf(leftRad) > MAX_WHEEL_SPEED_RAD) leftRad = (leftRad>0?MAX_WHEEL_SPEED_RAD:-MAX_WHEEL_SPEED_RAD);
    if (fabsf(rightRad) > MAX_WHEEL_SPEED_RAD) rightRad = (rightRad>0?MAX_WHEEL_SPEED_RAD:-MAX_WHEEL_SPEED_RAD);
    wheelSpeedRefL = leftRad;
    wheelSpeedRefR = rightRad;
    g_lastWheelCmdMs = millis();
    g_ctrlMode = CTRL_MODE_WHEEL_SPEED_OPEN;
}

void WheelSpeedTask(void *arg){
    TickType_t last = xTaskGetTickCount();
    static float rampRefL = 0.0f, rampRefR = 0.0f; // 斜坡后的参考
    while (1){
        uint32_t now = millis();
        // 超时回退
        if (g_ctrlMode != CTRL_MODE_YAW && (now - g_lastWheelCmdMs) > WHEEL_CMD_TIMEOUT_MS){
            wheelSpeedRefL = wheelSpeedRefR = 0.0f;
            rampRefL = rampRefR = 0.0f;
            g_ctrlMode = CTRL_MODE_YAW;
        }

        if (g_ctrlMode == CTRL_MODE_WHEEL_SPEED_CLOSED || g_ctrlMode == CTRL_MODE_WHEEL_SPEED_OPEN){
            // 参考斜坡限制
            rampRefL = slewLimit((float)wheelSpeedRefL, rampRefL, REF_SLEW_STEP);
            rampRefR = slewLimit((float)wheelSpeedRefR, rampRefR, REF_SLEW_STEP);

            // 读取当前速度 (rad/s)
            float curL = leftWheel.speed;
            float curR = rightWheel.speed;
            float torqueL = 0.0f, torqueR = 0.0f;

            if (g_ctrlMode == CTRL_MODE_WHEEL_SPEED_CLOSED){
                PID_SingleCalc(&wheelPidL, rampRefL, curL);
                PID_SingleCalc(&wheelPidR, rampRefR, curR);
                torqueL = wheelPidL.output;
                torqueR = wheelPidR.output;
            } else { // OPEN LOOP
                // 简单前馈 + 线性项 (无速度反馈闭环)
                if (fabsf(rampRefL) > 0.01f)
                    torqueL = OPEN_K_LINEAR * rampRefL + OPEN_K_BIAS * (rampRefL>0?1:-1);
                if (fabsf(rampRefR) > 0.01f)
                    torqueR = OPEN_K_LINEAR * rampRefR + OPEN_K_BIAS * (rampRefR>0?1:-1);
            }

            // 扭矩限幅（根据最大驱动电压反推可安全扭矩范围）
            float maxT = leftWheel.maxVoltage * leftWheel.torqueRatio * motorOutRatio;
            if (torqueL > maxT) torqueL = maxT; else if (torqueL < -maxT) torqueL = -maxT;
            if (torqueR > maxT) torqueR = maxT; else if (torqueR < -maxT) torqueR = -maxT;

            Motor_SetTorque(&leftWheel, torqueL);
            Motor_SetTorque(&rightWheel, torqueR);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(WHEEL_CTRL_PERIOD_MS));
    }
}

void WheelSpeedCtrl_Init(void){
    PID_Init(&wheelPidL, WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD, WHEEL_PID_MAX_SUM, WHEEL_PID_MAX_OUT);
    PID_Init(&wheelPidR, WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD, WHEEL_PID_MAX_SUM, WHEEL_PID_MAX_OUT);
    xTaskCreate(WheelSpeedTask, "WheelSpeedTask", 3072, NULL, 3, &wheelSpeedTaskHandle);

}
