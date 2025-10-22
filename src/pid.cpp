#include "pid.h"
#include <Arduino.h>
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

void PID_Init(PID *pid, float P, float I, float D, float ramp, float limit)
{
	pid->P = P;
	pid->I = I;
	pid->D = D;
	pid->ramp = ramp;
	pid->limit = limit;
	pid->error_prev = 0.0f;
	pid->output_prev = 0.0f;
	pid->integral_prev = 0.0f;
	pid->timestamp_prev = micros();
}

void setPID(PID *pid,float P,float I,float D,float ramp,float limit) 
{
  pid->P=P;
  pid->I=I;
  pid->D=D;
  pid->ramp=ramp;
  pid->limit=limit;
}

float getPID(PID *pid, float error)
{
  // 计算两次循环中间的间隔时间
    unsigned long timestamp_now = micros();
    float Ts = (timestamp_now - pid->timestamp_prev) * 1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 2*1e-3f;
    
    // P环
    float proportional = pid->P * error;
    // Tustin 散点积分（I环）
    float integral = pid->integral_prev + pid->I*Ts*0.5f*(error + pid->error_prev);
    // integral = _constrain(integral, -pid->limit, pid->limit);
    // D环（微分环节）
    float derivative = pid->D*(error - pid->error_prev)/Ts;

  // 将P,I,D三环的计算值加起来 (暂不立刻限幅, 先做 anti-windup 判断)
  float unsat_output = proportional + integral + derivative;
  float output = unsat_output;
  if (output > pid->limit) output = pid->limit;
  else if (output < -pid->limit) output = -pid->limit;

  // 简单 Anti-Windup: 若发生饱和且误差将继续推动积分向饱和方向增长 -> 不更新 integral_prev
  bool saturated = (unsat_output != output);
  if (saturated) {
    // 判断误差与(输出方向)是否同号, 若同号则撤销本次积分更新(保持上一积分值)
    if ((error > 0 && output > 0) || (error < 0 && output < 0)) {
      integral = pid->integral_prev; // 复原
    }
  }

    if(pid->ramp > 0){
        // 对PID的变化速率进行限制
        float output_rate = (output - pid->output_prev)/Ts;
        if (output_rate > pid->ramp)
            output = pid->output_prev + pid->ramp*Ts;
        else if (output_rate < -pid->ramp)
            output = pid->output_prev - pid->ramp*Ts;
    }
    // 保存值（为了下一次循环）
  pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
    pid->timestamp_prev = timestamp_now;
    return output;
}


