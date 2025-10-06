#include "adc.h"
#include <Adafruit_ADS1X15.h>
#include "motor.h"


float sourceVoltage = 12; //当前电源电压
void Batteryvoltage_Task(void *pvParameters)
{
     while (1)
     {
        sourceVoltage = analogRead(0) / 4095.0f * 3.3f * 11 * 12 / 13.5f;
        if(sourceVoltage < 8)
            sourceVoltage = 12;
            motorOutRatio = (12 - sourceVoltage) / 10.0f + 0.7f;
            vTaskDelay(100);
        }
}
void ADS1115_Init(void)
{
    xTaskCreate(Batteryvoltage_Task, "Batteryvoltage_Task", 4096, NULL, 5, NULL);
}