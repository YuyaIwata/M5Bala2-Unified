#include "imu_filter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <M5Unified.h>
#include "MadgwickAHRS.h"


static volatile float angle;
SemaphoreHandle_t angle_lock = NULL;
static void ImuUpdateTask(void *arg);

float gryo_x_offset;
float gryo_y_offset;
float gryo_z_offset;

static float acc_x_2, acc_y_2, acc_z_2;
// maybe not need
typedef struct {
  float a1;
  float a2;

  float b0;
  float b1;
  float b2;

  float y1;
  float y2;

  float x1;
  float x2;

} Butter_t;

float ButterFilter2(Butter_t* butter, float input) {
  float output;
  output = butter->b0 * input - butter->a2 * butter->y2 + butter->b1 * butter->x1 + butter->b2 * butter->x2 - butter->a1 * butter->y1 ;
  butter->y2 = butter->y1;
  butter->y1 = output;
  butter->x2 = butter->x1;
  butter->x1 = input;
  return output;
}

float getAngle() {
  float angle_out;

  if (angle_lock == NULL) {
    return 0;
  }

  xSemaphoreTake(angle_lock, portMAX_DELAY);
  angle_out = angle;
  xSemaphoreGive(angle_lock);
  return angle_out;
}

void ImuTaskStart(float x_offset, float y_offset, float z_offset, SemaphoreHandle_t *i2c_lock) {
  gryo_x_offset = 0 - x_offset;
  gryo_y_offset = 0 - y_offset;
  gryo_z_offset = 0 - z_offset;
  xTaskCreatePinnedToCore(ImuUpdateTask, "imu_task", 4 * 1024, i2c_lock, 5, NULL, 1);
}

void ImuUpdateTask(void *arg) {
  uint32_t last_ticks = xTaskGetTickCount();

  float acc_x = 0, acc_y = 0, acc_z = 0;
  float gyro_x, gyro_y, gyro_z;
  float yaw_ahrs = 0, pitch_ahrs = 0, roll_ahrs = 0;

  m5::IMU_Class::imu_data_t imu_data;
  SemaphoreHandle_t i2c_lock = *(SemaphoreHandle_t*)arg;
  angle_lock = xSemaphoreCreateRecursiveMutex();

  bool fast_to_normal_angle = true;
  uint32_t last_usec = 0;

  Butter_t butter_acc_x;
  Butter_t butter_acc_y;
  Butter_t butter_acc_z;

  memset(&butter_acc_x, 0, sizeof(Butter_t));
  memset(&butter_acc_y, 0, sizeof(Butter_t));
  memset(&butter_acc_z, 0, sizeof(Butter_t));

  // 250hz in, filter 50hz, low pass, butterworth.
  // M5Unified drives the MPU6886 at 250Hz (SMPLRT_DIV=3), where the original
  // 500Hz coefficients would have halved the cutoff to 25Hz and added lag.
  // Bilinear transform, w = tan(pi * fc / fs):
  //   b0 = b2 = w^2/a, b1 = 2*w^2/a, a1 = 2*(w^2-1)/a, a2 = (1-sqrt(2)w+w^2)/a
  //   where a = 1 + sqrt(2)*w + w^2
  butter_acc_x.a1 = -0.369527f;
  butter_acc_x.a2 = 0.195816f;
  butter_acc_x.b0 = 0.206572f;
  butter_acc_x.b1 = 0.413144f;
  butter_acc_x.b2 = 0.206572f;

  memcpy(&butter_acc_y, &butter_acc_x, sizeof(Butter_t));
  memcpy(&butter_acc_z, &butter_acc_x, sizeof(Butter_t));

  // make sure angle fast to normal
  MadgwickAHRSetBeta(10);

  for(;;) {
    // M5Unified polls the sensor instead of draining a FIFO burst, so the loop
    // runs faster than the 250Hz data rate and only integrates on fresh data.
    xSemaphoreTake(i2c_lock, portMAX_DELAY);
    bool updated = (M5.Imu.update() != 0);
    if (updated) {
      M5.Imu.getImuData(&imu_data);
    }
    xSemaphoreGive(i2c_lock);

    if (updated) {
      if (last_usec != 0) {
        float delta_t = (float)(imu_data.usec - last_usec) * 1e-6f;
        // Ignore implausible intervals (first sample, scheduling hiccups)
        if (delta_t > 0.0f && delta_t < 0.1f) {
          MadgwickAHRSetDeltaT(delta_t);
        }
      }
      last_usec = imu_data.usec;

      // M5Unified reports accel in g and gyro in deg/s, the same units the
      // original raw-register conversion produced.
      acc_x = imu_data.accel.x;
      acc_y = imu_data.accel.y;
      acc_z = imu_data.accel.z;
      gyro_x = imu_data.gyro.x + gryo_x_offset;
      gyro_y = imu_data.gyro.y + gryo_y_offset;
      gyro_z = imu_data.gyro.z + gryo_z_offset;

      // Don't know if this is necessary
      acc_x_2 = ButterFilter2(&butter_acc_x, acc_x);
      acc_y_2 = ButterFilter2(&butter_acc_y, acc_y);
      acc_z_2 = ButterFilter2(&butter_acc_z, acc_z);
      MadgwickAHRSupdateIMU(gyro_x * DEG_TO_RAD, gyro_y * DEG_TO_RAD, gyro_z * DEG_TO_RAD, acc_x_2, acc_y_2, acc_z_2, &pitch_ahrs, &roll_ahrs, &yaw_ahrs);

      if (fast_to_normal_angle) {
        int16_t roll = atan2(acc_y, sqrt(acc_z * acc_z + acc_x * acc_x)) * 360.0 / 2.0 / PI;

        if(fabs(roll - roll_ahrs) < 1 && roll != roll_ahrs) {
          fast_to_normal_angle = false;
          MadgwickAHRSetBeta(0.1);
        }
      }

      xSemaphoreTake(angle_lock, portMAX_DELAY);
      if (!fast_to_normal_angle) {
        angle = roll_ahrs;
      }
      xSemaphoreGive(angle_lock);
    }

    vTaskDelayUntil(&last_ticks, pdMS_TO_TICKS(2));
  }
}
