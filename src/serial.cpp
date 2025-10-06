#include <Arduino.h>
#include "serial.h"
#include "imu.h"
#include "motor.h"
#include "ctrl.h"
#include "adc.h"
#include "pid.h"

#define MAX_SPEED           (0.6F)
#define MAX_YAWSPEED        (0.5F)
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
static void StopLinearTask(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(2500)); 
    target.speedCmd = 0;
    target.yawSpeedCmd = 0;               
    Serial.printf("�Զ�ֹͣ�˶�\n");
    vTaskDelete(NULL);               
}


void UART2_CommandTask(void *pvParameters) {
    char rxBuffer[128] = {0};  
    int bufIndex = 0;
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {

        while (Serial2.available() > 0) {
            char c = Serial2.read();

            if (c == '\n' || bufIndex >= 127) {
                rxBuffer[bufIndex] = '\0';
                Serial.printf("%s\n", rxBuffer);
                bufIndex = 0;

                char *cmd = strtok(rxBuffer, ",");  
                if (!cmd) continue;

                if (strcmp(cmd, "GoForward") == 0) {  
                    target.speedCmd = 0.3*MAX_SPEED;
                    Serial.printf("��ǰ��\n");
                    xTaskCreate(StopLinearTask, "StopLinearTask", 4096, NULL, 1, NULL);  // ����
                }
                else if (strcmp(cmd, "GoBack") == 0) { 
                    target.speedCmd = -0.3*MAX_SPEED;
                    Serial.printf("�����\n");
                    xTaskCreate(StopLinearTask, "StopLinearTask", 4096, NULL, 1, NULL);  
                }
                else if (strcmp(cmd, "TurnLeft") == 0) { 
                    target.yawSpeedCmd = MAX_YAWSPEED;
                    Serial.printf("����ת\n");
                    xTaskCreate(StopLinearTask, "StopLinearTask", 4096, NULL, 1, NULL);  
                }
                else if (strcmp(cmd, "TurnRight") == 0) {  
                    target.yawSpeedCmd = -MAX_YAWSPEED;
                    Serial.printf("����ת\n");
                    xTaskCreate(StopLinearTask, "StopLinearTask", 4096, NULL, 1, NULL); 
                }
                else if (strcmp(cmd, "Jump") == 0) {  
                    Serial.printf("��Ծ\n");
                }
                else if (strcmp(cmd, "CrossLeg") == 0) { 
                    Serial.printf("�������л�\n");
                }
                else if (strcmp(cmd, "STOP") == 0) {  
                    target.speedCmd = 0;
                    target.yawSpeedCmd = 0;
                    Serial.printf("ֹͣ\n");
                }
                else {  
                    char *sequence = rxBuffer; 
                    char *saveptr; 
                    char *subCmd = strtok_r(sequence, ";", &saveptr);  

                    while (subCmd != NULL) {
                        Serial.printf("������ָ��: %s\n", subCmd); 
                        char *subType = strtok(subCmd, ",");
                        char *subParam = strtok(NULL, ",");   


                        char *tempSaveptr = saveptr; 
                        char *nextSubCmd = strtok_r(NULL, ";", &tempSaveptr);  
                        bool isLast = (nextSubCmd == NULL);  

                        if (subType != NULL) {
                            if (strcmp(subType, "GoForward") == 0) {
                                target.speedCmd = 0.3*MAX_SPEED;
                                Serial.printf("��ǰ��\n");
                                xTaskCreate(StopLinearTask, "StopLinearTask", 4096, NULL, 1, NULL);
                            }
                            else if (strcmp(subType, "GoBack") == 0) {
                                target.speedCmd =-0.3*MAX_SPEED;
                                Serial.printf("�����\n");
                                xTaskCreate(StopLinearTask, "StopLinearTask", 4096, NULL, 1, NULL);  
                            }
                            else if (strcmp(subType, "TurnLeft") == 0) {
                                target.yawSpeedCmd = MAX_YAWSPEED;
                                Serial.printf("����ת\n");
                                xTaskCreate(StopLinearTask, "StopLinearTask", 4096, NULL, 1, NULL); 
                            }
                            else if (strcmp(subType, "TurnRight") == 0) {
                                target.yawSpeedCmd = -MAX_YAWSPEED;
                                Serial.printf("����ת\n");
                                xTaskCreate(StopLinearTask, "StopLinearTask", 4096, NULL, 1, NULL);  
                            }
                            else if (strcmp(subType, "Jump") == 0) {
                                Serial.printf("��Ծ\n");
                            }
                            else if (strcmp(subType, "StandUp") == 0) {

                            }
                            else if (strcmp(subType, "CrossLeg") == 0) {
                                xTaskCreate(StopLinearTask, "StopLinearTask", 4096, NULL, 1, NULL);                          
                                Serial.printf("�������л�\n");
                            }
                            else if (strcmp(subType, "STOP") == 0) {
                                target.speedCmd = 0;
                                target.yawSpeedCmd = 0;
                                
                                Serial.printf("ֹͣ\n");
                            }
                            else if (strcmp(subType, "IncreaseLegLength") == 0) {
                                Serial.printf("�����ȳ�\n");
                            }
                            else if (strcmp(subType, "DecreaseLegLength") == 0) {
                                Serial.printf("�����ȳ�\n");
                            }
                        }

                        subCmd = strtok_r(NULL, ";", &saveptr);  

                    }
                }

            }  
            else {  
                rxBuffer[bufIndex++] = c; 
            }
        }
        
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10)); 
    }
}

void Serial_Task(void *pvParameters)
{
    while (1)
    {
        //Serial.printf("%.3f\n",leftWheel.angle);	
        // Serial.printf("%.3f\n",rightWheel.angle);	
        //Serial.printf("%.3f,%.3f\r\n",leftWheel.voltage,rightWheel.voltage);
        // Serial.printf("%f,%f\r\n",stateVar.x,stateVar.dx);
        //Serial.printf("%.3f,%.3f,%.3f\r\n",target.yawAngle, imuData.yaw,yawPID.output);
        //Serial.printf("%.3f,%.3f\n",vot,motorOutRatio);
        //Serial.printf("%.3f\n",leftWheel.angle);	
        //Serial.printf("%f\n",leftJoint[0].speed);	
        // Serial.printf("%f,%f,%f\n",imuData.pitchSpd,imuData.rollSpd,imuData.yawSpd); 	//�鿴���ٶ� ȷ��IMU����
        // Serial2.printf("%f,%f,%f\n",imuData.pitch,imuData.roll,imuData.yaw); 	
        // Serial2.printf("%f\n",imuData.pitch); 	
        // Serial2.printf("%f,%f,%f\n",imuData.pitch,imuData.roll,imuData.yaw); 	
        // Serial.printf("%f,%f,%f\n",imuData.pitch,imuData.roll,imuData.yaw); 	
        //Serial.printf("Hello\r\n");
        vTaskDelay(50);
    }
} 

void Serial_Init(void)
{
    Serial.begin(115200);        
    Serial.setTimeout(10);
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    xTaskCreate(UART2_CommandTask, "UART2_CommandTask", 4096, NULL, 1, NULL);
    xTaskCreate(Serial_Task, "Serial_Task", 4096, NULL, 1, NULL);
}
