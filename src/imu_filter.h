#ifndef __IMU__H_
#define __IMU__H_

#include "Arduino.h"

float getAngle();

// Snapshot of the estimator, taken under the same lock that guards the angle so
// that all fields belong to one IMU sample.
typedef struct {
  float quat_w, quat_x, quat_y, quat_z;  // orientation from the Madgwick filter
  float gyro_x, gyro_y, gyro_z;          // deg/s, offset corrected
  float accel_x, accel_y, accel_z;       // g
  float angle;                           // roll used by the balance controller, degrees
  uint32_t usec;                         // timestamp of the underlying sample
} ImuState_t;

void getImuState(ImuState_t* state);

// Offsets are gyro rates in deg/s, as reported by M5Unified.
void ImuTaskStart(float x_offset, float y_offset, float z_offset, SemaphoreHandle_t* i2c_lock);

#endif
