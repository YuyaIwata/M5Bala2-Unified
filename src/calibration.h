#pragma once

void calibrationGryo();
void calibrationInit();
void calibrationSaveCenterAngle(float angle);

// Gyro offsets are in deg/s, matching the units M5Unified reports.
void calibrationGet(float* gyro_x_offset, float* gyro_y_offset, float* gyro_z_offset, float *angle_center);
