#ifndef COMMANDS_H
#define COMMANDS_H
#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'   
#define MOTOR_SPEEDS_DEPRECATED   'm'  
#define MOTOR_RAW_ANGLE  'g'  // 返回左右轮 rawAngle（原始角度）
#define MOTOR_ANGLE      'h'  // 返回左右轮 angle（校正后角度）
#define MOTOR_SPEED      'j'  // 返回左右轮 speed（rad/s）
// 新增差速控制相关：
#define DIFF_SET_VW      'v'  // 设置线速度(m/s) 与角速度(rad/s) 参考（进入速度模式闭环）
#define DIFF_QUERY_STATE 'q'  // 查询当前底盘状态（里程/轮速/航向/参考值/模式）
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define SYSTEM_INFO    'i'   // 系统状态信息
#define DIAGNOSTICS    'z'   // 诊断信息
#define SELF_TEST      'y'   // 自检

#define LEFT            0
#define RIGHT           1

#endif
