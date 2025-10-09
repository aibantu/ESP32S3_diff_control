#include "AS5600.h"
#include "Wire.h"
#include <Arduino.h> 

#define _2PI 6.28318530718f



// AS5600 相关
// 范围是0到6.28
// 该函数通过 I2C 通信从 AS5600 传感器读取 12 位的角度数据，对高低字节进行处理以提取有效位，然后将原始数据转换为以弧度为单位的角度值并返回。
double Sensor_AS5600::getSensorAngle() {
  // 该变量存储了 AS5600 传感器中角度数据高字节所在的寄存器地址，值为 0x0C
  uint8_t angle_reg_msb = 0x0C;

  // 这是一个长度为 2 的字节数组，用于存储从传感器读取的两个字节的数据，其中一个是高字节，另一个是低字节。
  byte readArray[2];
  uint16_t readValue = 0;  // 用于存储将两个字节合并后的 16 位整数值。

  wire->beginTransmission(0x36);  // 启动与 AS5600 传感器的 I2C 通信，0x36 是传感器的 I2C 地址。
  wire->write(angle_reg_msb);  // 向传感器发送要读取的寄存器地址（即角度数据的高字节地址）。
  wire->endTransmission(false);  // 结束当前的传输，但保持 I2C 总线的连接，以便后续请求数据。
  wire->requestFrom(0x36, (uint8_t)2);  // 向传感器请求 2 个字节的数据。

  // 依次读取这两个字节的数据，并将它们存储在 readArray 数组中。
  for (byte i=0; i < 2; i++) {          // 
    readArray[i] = wire->read();
  }
  int _bit_resolution=12;  // 表示传感器的分辨率为 12 位，即角度数据的精度为 12 位二进制数。

  /**
   * 寄存器的位编号：寄存器的位编号从 0 开始，最右边的位是第 0 位，最左边的位是第 7 位。
   * 12 位数据的分布：假设 12 位数据的最高位是第 11 位，最低位是第 0 位。
   * 而高字节寄存器涵盖的是从第 7 位到第 11 位。
   * 计算有效位数量：所以高字节寄存器里有效位的数量就是 11 - 7 = 4 位。
   */
  int _bits_used_msb=11-7;  // 高字节寄存器里有效位的数量

  // 计算传感器的一圈总计数（Counts Per Revolution），即 2 的 _bit_resolution 次方。
  // 对于 12 位分辨率的传感器，cpr 的值为 4096。
  float cpr = pow(2, _bit_resolution);

  // // 计算低字节中实际使用的位数，即总分辨率减去高字节中使用的位数。
  int lsb_used = _bit_resolution - _bits_used_msb;   // 12-4 = 8

  // 用于提取低字节中的有效位。通过左移操作和减法运算创建一个掩码，该掩码只保留低字节中实际使用的位。
  uint8_t lsb_mask = (uint8_t)( (2 << lsb_used) - 1 );

  // 用于提取高字节中的有效位，同样通过左移和减法运算创建。
  uint8_t msb_mask = (uint8_t)( (2 << _bits_used_msb) - 1 );
  readValue = ( readArray[1] &  lsb_mask ); // 提取低字节中的有效位，并将其赋值给 readValue。

  // 提取高字节中的有效位，并将其左移 lsb_used 位，然后加到 readValue 上，完成高低字节的合并。
  readValue += ( ( readArray[0] & msb_mask ) << lsb_used );

  // 将合并后的 16 位整数值转换为 0 到 1 之间的小数，表示当前角度在一圈中所占的比例。
  return (readValue/ (float)cpr) * _2PI;  // 单位是弧度，范围是0到2pi

}

//AS5600 相关

//=========角度处理相关=============
Sensor_AS5600::Sensor_AS5600(int Mot_Num) {
   _Mot_Num=Mot_Num;  //使得 Mot_Num 可以统一在该文件调用
   
}
void Sensor_AS5600::Sensor_init(TwoWire* _wire) {
    wire=_wire;
    wire->begin();   //电机Sensor
    delay(500);
    getSensorAngle(); 
    delayMicroseconds(1);
    prev_angle = getSensorAngle(); 
    last_time = micros();
    delay(1);
    getSensorAngle(); 
    delayMicroseconds(1);
    current_angle = getSensorAngle(); 
    current_time = micros();
}


void Sensor_AS5600::Sensor_update() {
    float new_angle = getSensorAngle();
    current_time = micros();
    float d_angle = new_angle - current_angle;
    // 圈数检测
    if(abs(d_angle) > (0.8f*_2PI) )
    {
        full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    }
    current_angle = new_angle;
}

float Sensor_AS5600::getMechanicalAngle() {
    return current_angle;
}

float Sensor_AS5600::getAngle(){
    return (float)full_rotations * _2PI + current_angle;
}

float Sensor_AS5600::getVelocity() {
    // 计算采样时间
    float Ts = (current_time - last_time)*1e-6;
    // 快速修复奇怪的情况（微溢出）
    if(Ts <= 0) Ts = 1e-3f;
    // 速度计算
    float vel = ( (float)(full_rotations - last_full_rotations)*_2PI + (current_angle - prev_angle) ) / Ts;    
    // 保存变量以待将来使用
    prev_angle = current_angle;
    last_full_rotations = full_rotations;
    last_time = current_time;
    return vel;//返回计算的角速度值，弧度单位
}
