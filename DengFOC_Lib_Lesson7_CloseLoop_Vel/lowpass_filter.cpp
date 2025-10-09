#include "lowpass_filter.h"

LowPassFilter::LowPassFilter(float time_constant): Tf(time_constant), last_val(0.0f)
{
    last_timestamp = micros();
}


float LowPassFilter::operator() (float x)
{
    unsigned long timestamp = micros();
    float dt = (timestamp - last_timestamp)*1e-6f;

    if (dt < 0.0f ) dt = 2.0f*1e-3f;
    else if(dt > 0.3f) {
        last_val = x;
        last_timestamp = timestamp;
        return x;
    }

    float alpha = Tf/(Tf + dt);
    float y = alpha*last_val + (1.0f - alpha)*x;
    last_val = y;
    last_timestamp = timestamp;
    return y;
}
