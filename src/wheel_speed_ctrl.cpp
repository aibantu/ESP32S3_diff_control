#include "wheel_speed_ctrl.h"
#include <math.h>

// 任务句柄（对外提供）
TaskHandle_t wheelSpeedTaskHandle = NULL;
// 仅保留轮速控制参考（不再有全局模式切换）
volatile uint32_t g_lastWheelCmdMs = 0;
volatile float wheelSpeedRefL = 0.0f;
volatile float wheelSpeedRefR = 0.0f;

static PID wheelPidL, wheelPidR;

// 配置常量
#define WHEEL_PID_KP 0.9f
#define WHEEL_PID_KI 0.0f
// 调整：周期从8ms改为4ms，为保持等效微分增益，将KD加倍（隐式dt实现）
#define WHEEL_PID_KD 0.06f
#define WHEEL_PID_MAX_SUM 0.5f
#define WHEEL_PID_MAX_OUT 0.9f

// 控制周期与 CtrlBasic_Task 统一为 4ms（原 8ms）
#define WHEEL_CTRL_PERIOD_MS 4
#define WHEEL_CMD_TIMEOUT_MS 5000   // 超时5秒：仅清零速度，不回退模式
#define MAX_WHEEL_SPEED_RAD 25.0f     // 速度参考限幅
// 斜坡步长：原 8ms 周期为 2 rad/s/周期 => 等效加速度约 250 rad/s^2
// 改为 4ms 后为保持相同加速度，将每周期增量减半为 1 rad/s
#define REF_SLEW_STEP (1.0f)          // 每周期最大参考增量(rad/s)（可调，隐式dt）

// 开环参数
#define OPEN_K_LINEAR 0.03f
#define OPEN_K_BIAS   0.08f

static float slewLimit(float target, float current, float step){
    float d = target - current;
    if (d > step) d = step; else if (d < -step) d = -step;
    return current + d;
}

static bool isClosedLoop = true; // 默认闭环

void WheelSpeedCtrl_SetClosedLoop(float leftRad, float rightRad){
    if (fabsf(leftRad) > MAX_WHEEL_SPEED_RAD) leftRad = (leftRad>0?MAX_WHEEL_SPEED_RAD:-MAX_WHEEL_SPEED_RAD);
    if (fabsf(rightRad) > MAX_WHEEL_SPEED_RAD) rightRad = (rightRad>0?MAX_WHEEL_SPEED_RAD:-MAX_WHEEL_SPEED_RAD);
    wheelSpeedRefL = leftRad;
    wheelSpeedRefR = rightRad;
    g_lastWheelCmdMs = millis();
    isClosedLoop = true;
}

void WheelSpeedCtrl_SetOpenLoop(float leftRad, float rightRad){
    if (fabsf(leftRad) > MAX_WHEEL_SPEED_RAD) leftRad = (leftRad>0?MAX_WHEEL_SPEED_RAD:-MAX_WHEEL_SPEED_RAD);
    if (fabsf(rightRad) > MAX_WHEEL_SPEED_RAD) rightRad = (rightRad>0?MAX_WHEEL_SPEED_RAD:-MAX_WHEEL_SPEED_RAD);
    wheelSpeedRefL = leftRad;
    wheelSpeedRefR = rightRad;
    g_lastWheelCmdMs = millis();
    isClosedLoop = false;
}

void WheelSpeedTask(void *arg){
    TickType_t last = xTaskGetTickCount();
    static float rampRefL = 0.0f, rampRefR = 0.0f; // 斜坡后的参考
    while (1){
        uint32_t now = millis();
        // 超时处理：5秒无新指令 -> 设定参考为0（停止）
        if ((now - g_lastWheelCmdMs) > WHEEL_CMD_TIMEOUT_MS){
            wheelSpeedRefL = wheelSpeedRefR = 0.0f;
            rampRefL = rampRefR = 0.0f; // 立即停止，不使用缓降
        }

        // 参考斜坡限制（始终执行）
        rampRefL = slewLimit((float)wheelSpeedRefL, rampRefL, REF_SLEW_STEP);
        rampRefR = slewLimit((float)wheelSpeedRefR, rampRefR, REF_SLEW_STEP);

        // 读取当前速度 (rad/s)
        float curL = leftWheel.speed;
        float curR = rightWheel.speed;
        float torqueL = 0.0f, torqueR = 0.0f;

        if (isClosedLoop){
            PID_SingleCalc(&wheelPidL, rampRefL, curL);
            PID_SingleCalc(&wheelPidR, rampRefR, curR);
            torqueL = wheelPidL.output;
            torqueR = wheelPidR.output;
        } else { // OPEN LOOP
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

        vTaskDelayUntil(&last, pdMS_TO_TICKS(WHEEL_CTRL_PERIOD_MS));
    }
}

void WheelSpeedCtrl_Init(void){
    PID_Init(&wheelPidL, WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD, WHEEL_PID_MAX_SUM, WHEEL_PID_MAX_OUT);
    PID_Init(&wheelPidR, WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD, WHEEL_PID_MAX_SUM, WHEEL_PID_MAX_OUT);
    xTaskCreate(WheelSpeedTask, "WheelSpeedTask", 3072, NULL, 3, &wheelSpeedTaskHandle);

}
