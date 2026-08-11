#include "calibration.h"
#include <M5Unified.h>
#include <Preferences.h>

Preferences preferences;

// M5Unified exposes calibrated float rates instead of raw ADC counts, so the
// offsets are stored as deg/s under new keys. The old "gryo_*" integer keys are
// left untouched and simply ignored.
static const uint16_t CALIBRATION_SAMPLES = 500;  // 500 samples @250Hz = 2 seconds
static const uint32_t CALIBRATION_SETTLE_MS = 2000;
// Sensor noise alone spans well under 2 deg/s peak-to-peak at rest, so a wider
// spread means the robot was moved and the average would carry that motion.
static const float CALIBRATION_MAX_SPREAD_DPS = 6.0f;
static const uint8_t CALIBRATION_MAX_RETRIES = 5;

static bool SampleGyroAverage(uint16_t times, float* x_offset, float* y_offset, float* z_offset) {
  double x = 0, y = 0, z = 0;
  float gyro_x, gyro_y, gyro_z;
  float lo[3] = {1e9f, 1e9f, 1e9f};
  float hi[3] = {-1e9f, -1e9f, -1e9f};
  uint16_t taken = 0;

  while (taken < times) {
    // Only accumulate fresh samples; update() returns 0 between data-ready events.
    if (M5.Imu.update()) {
      M5.Imu.getGyro(&gyro_x, &gyro_y, &gyro_z);
      x += gyro_x;
      y += gyro_y;
      z += gyro_z;

      const float v[3] = {gyro_x, gyro_y, gyro_z};
      for (int i = 0; i < 3; i++) {
        if (v[i] < lo[i]) { lo[i] = v[i]; }
        if (v[i] > hi[i]) { hi[i] = v[i]; }
      }
      taken++;
    }
    delay(1);
  }

  *x_offset = x / times;
  *y_offset = y / times;
  *z_offset = z / times;

  for (int i = 0; i < 3; i++) {
    if (hi[i] - lo[i] > CALIBRATION_MAX_SPREAD_DPS) {
      return false;
    }
  }
  return true;
}

// Averaging whatever the gyro reports the moment the sketch boots picks up the
// motion of releasing the button, which lands in the average as a bias of
// several deg/s. Wait for the robot to be put down, then reject any window that
// still contains movement.
bool GetGyroOffset(uint16_t times, float* x_offset, float* y_offset, float* z_offset) {
  delay(CALIBRATION_SETTLE_MS);

  for (uint8_t attempt = 0; attempt < CALIBRATION_MAX_RETRIES; attempt++) {
    if (SampleGyroAverage(times, x_offset, y_offset, z_offset)) {
      return true;
    }
    M5.Lcd.printf("Movement detected, retrying...\r\n");
  }
  return false;
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

  M5.Lcd.printf("Release the button, put BALA2 down\r\n");
  M5.Lcd.printf("and keep it still...\r\n");

  float x_offset = 0;
  float y_offset = 0;
  float z_offset = 0;
  bool ok = GetGyroOffset(CALIBRATION_SAMPLES, &x_offset, &y_offset, &z_offset);

  if (!ok) {
    M5.Lcd.setTextColor(RED);
    M5.Lcd.printf("Calibration FAILED (kept moving)\r\n");
    M5.Lcd.printf("Previous values are kept.\r\n");
    Serial.printf("calibration failed: motion detected\n");
    M5.Lcd.setTextColor(GREEN);
    delay(3000);
    return;
  }

  M5.Lcd.printf("Finish calibration !!!\r\n");
  Serial.printf("calibration: x=%.4f y=%.4f z=%.4f\n", x_offset, y_offset, z_offset);

  preferences.putFloat("gyro_x", x_offset);
  preferences.putFloat("gyro_y", y_offset);
  preferences.putFloat("gyro_z", z_offset);

  delay(1000);
}

void calibrationSaveCenterAngle(float angle) {
  preferences.putFloat("angle", angle);
}
