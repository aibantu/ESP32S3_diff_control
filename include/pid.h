#ifndef PID_H
#define PID_H
typedef struct 
{	float P;
	float I;			   // rad/s
	float D;  // rad
	float ramp;
    float limit;
    float error_prev; //!< 最后的跟踪误差值
    float output_prev;  //!< 最后一个 pid 输出值
    float integral_prev; //!< 最后一个积分分量值
    unsigned long timestamp_prev; //!< 上次执行时间戳
} PID;

void PID_Init(PID *pid, float P, float I, float D, float ramp, float limit);
void setPID(PID *pid,float P,float I,float D,float ramp,float limit);
float getPID(PID *pid, float error);
#endif
