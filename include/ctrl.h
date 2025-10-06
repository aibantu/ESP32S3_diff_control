#ifndef CTRL_H
#define CTRL_H

#include "stdio.h"
#include "pid.h"
typedef struct 
{
   	float position;	 // m
	float speedCmd;	 // m/s
	float speed;    // m/s
	float yawSpeedCmd; // rad/s
	float yawAngle;	 // rad
	float rollAngle; // rad
	float target; // m
}Target;


typedef struct 
{
	float x, dx;
} StateVar;


extern Target target;
extern StateVar stateVar;

void Ctrl_Init(void);


#endif