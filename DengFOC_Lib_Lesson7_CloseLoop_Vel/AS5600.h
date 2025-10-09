#include <Arduino.h> 
#include "Wire.h"

class Sensor_AS5600
{
  public:
    Sensor_AS5600(int Mot_Num);
    void Sensor_init(TwoWire* _wire = &Wire);
    void Sensor_update();
    float getAngle();
    float getVelocity();
    float getMechanicalAngle();
    double getSensorAngle();
  private:
    int _Mot_Num;
    //AS5600 变量定义
    //int sensor_direction=1;       //编码器旋转方向定义
    float prev_angle= 0;
    float current_angle = 0;
    long current_time = 0;
    long last_time = 0;
    int32_t full_rotations=0; // 总圈数计数
    int32_t last_full_rotations=0; //用于速度计算的先前完整旋转圈数
TwoWire* wire;
};
