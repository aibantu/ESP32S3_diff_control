#include <Arduino.h> 

#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

/**
  * 低通滤波器类
  */
  
class LowPassFilter
{
public:
    LowPassFilter(float Tf); 
    ~LowPassFilter() = default;

    float operator() (float x);
    float Tf; // 低通滤波时间常数

protected:
    unsigned long last_timestamp;  // 最后执行时间戳
    float last_val; // 上一个循环中的过滤后的值
};

#endif
