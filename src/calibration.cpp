#include "calibration.h"
#include <M5Unified.h>
#include <Preferences.h>

Preferences preferences;

// M5Unified exposes calibrated float rates instead of raw ADC counts, so the
// offsets are stored as deg/s under new keys. The old "gryo_*" integer keys are
// left untouched and simply ignored.
static const uint16_t CALIBRATION_SAMPLES = 500;  // 500 samples @250Hz = 2 seconds

void GetGyroOffset(uint16_t times, float* x_offset, float* y_offset, float* z_offset) {
  double x = 0, y = 0, z = 0;
  float gyro_x, gyro_y, gyro_z;
  uint16_t taken = 0;

  while (taken < times) {
    // Only accumulate fresh samples; update() returns 0 between data-ready events.
    if (M5.Imu.update()) {
      M5.Imu.getGyro(&gyro_x, &gyro_y, &gyro_z);
      x += gyro_x;
      y += gyro_y;
      z += gyro_z;
      taken++;
    }
    delay(1);
  }

  *x_offset = x / times;
  *y_offset = y / times;
  *z_offset = z / times;
}

void calibrationInit() {
  preferences.begin("Bala2Cal", false);
}

void calibrationGet(float* gyro_x_offset, float* gyro_y_offset, float* gyro_z_offset, float *angle_center) {
  *gyro_x_offset = preferences.getFloat("gyro_x", 0.0f);
  *gyro_y_offset = preferences.getFloat("gyro_y", 0.0f);
  *gyro_z_offset = preferences.getFloat("gyro_z", 0.0f);

  *angle_center = preferences.getFloat("angle", 0.0);
}

void calibrationGryo() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextFont(2);
  M5.Lcd.setTextColor(GREEN);
  M5.Lcd.printf("Start gryo calibration\r\n");
  delay(1000);

  M5.Lcd.printf("Please keep BALA2 still for 2 seconds\r\n");

  float x_offset = 0;
  float y_offset = 0;
  float z_offset = 0;
  GetGyroOffset(CALIBRATION_SAMPLES, &x_offset, &y_offset, &z_offset);
  M5.Lcd.printf("Finish calibration !!!\r\n");

  preferences.putFloat("gyro_x", x_offset);
  preferences.putFloat("gyro_y", y_offset);
  preferences.putFloat("gyro_z", z_offset);

  delay(1000);
}

void calibrationSaveCenterAngle(float angle) {
  preferences.putFloat("angle", angle);
}
