#ifndef __IMU__H_
#define __IMU__H_

#include "Arduino.h"

float getAngle();

// Offsets are gyro rates in deg/s, as reported by M5Unified.
void ImuTaskStart(float x_offset, float y_offset, float z_offset, SemaphoreHandle_t* i2c_lock);

#endif
