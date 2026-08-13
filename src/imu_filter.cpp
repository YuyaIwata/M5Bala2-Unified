#include "debug_config.h"
#include "imu_filter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <M5Unified.h>
#include "MadgwickAHRS.h"


static volatile float angle;
SemaphoreHandle_t angle_lock = NULL;
static void ImuUpdateTask(void *arg);

// Published alongside the angle so that consumers (micro-ROS) get one coherent
// sample instead of separately locked reads.
static ImuState_t imu_state;

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

void getImuState(ImuState_t* state) {
  if (state == NULL) {
    return;
  }
  if (angle_lock == NULL) {
    memset(state, 0, sizeof(ImuState_t));
    state->quat_w = 1.0f;
    return;
  }

  xSemaphoreTake(angle_lock, portMAX_DELAY);
  *state = imu_state;
  xSemaphoreGive(angle_lock);
}

// M5Unified initialises the MPU6886 with SMPLRT_DIV=3 (250Hz). Every other
// register it writes matches what the M5Stack library used, but that library
// used SMPLRT_DIV=1 (500Hz), and the PID gains -- kd=90 in particular -- were
// tuned against an estimator fed at that rate. Restore 500Hz so the control
// dynamics match the tuning.
static void SetImuSampleRate500Hz() {
  constexpr uint8_t MPU6886_I2C_ADDR = 0x68;
  constexpr uint8_t REG_SMPLRT_DIV = 0x19;
  constexpr uint8_t SMPLRT_DIV_500HZ = 0x01;
  constexpr uint32_t IMU_I2C_FREQ = 400000;

  M5.In_I2C.writeRegister8(MPU6886_I2C_ADDR, REG_SMPLRT_DIV, SMPLRT_DIV_500HZ, IMU_I2C_FREQ);

#if BALA_DEBUG_TELEMETRY
  uint8_t readback = M5.In_I2C.readRegister8(MPU6886_I2C_ADDR, REG_SMPLRT_DIV, IMU_I2C_FREQ);
  Serial.printf("[imu] SMPLRT_DIV=%u (expect %u)\n", readback, SMPLRT_DIV_500HZ);
#endif
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

  xSemaphoreTake(i2c_lock, portMAX_DELAY);
  SetImuSampleRate500Hz();
  xSemaphoreGive(i2c_lock);

  bool fast_to_normal_angle = true;
  uint32_t last_usec = 0;

  Butter_t butter_acc_x;
  Butter_t butter_acc_y;
  Butter_t butter_acc_z;

  memset(&butter_acc_x, 0, sizeof(Butter_t));
  memset(&butter_acc_y, 0, sizeof(Butter_t));
  memset(&butter_acc_z, 0, sizeof(Butter_t));

  // 500hz in, filter 50hz, low pass, butterworth
  butter_acc_x.a1 = -1.142980f;
  butter_acc_x.a2 = 0.412801f;
  butter_acc_x.b0 = 0.067455f;
  butter_acc_x.b1 = 0.134910f;
  butter_acc_x.b2 = 0.067455f;

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
      MadgwickAHRSGetQuaternion(&imu_state.quat_w, &imu_state.quat_x,
                                &imu_state.quat_y, &imu_state.quat_z);
      imu_state.gyro_x = gyro_x;
      imu_state.gyro_y = gyro_y;
      imu_state.gyro_z = gyro_z;
      imu_state.accel_x = acc_x;
      imu_state.accel_y = acc_y;
      imu_state.accel_z = acc_z;
      imu_state.angle = angle;
      imu_state.usec = imu_data.usec;
      xSemaphoreGive(angle_lock);

#if BALA_DEBUG_TELEMETRY
      // Report the achieved IMU rate once a second so the Madgwick integration
      // interval can be checked against the sensor's actual output data rate.
      static uint32_t sample_count = 0;
      static uint32_t rate_window_start = 0;
      sample_count++;
      uint32_t now = millis();
      if (now - rate_window_start >= 1000) {
        Serial.printf("[imu] t=%6lums rate=%3lu Hz raw=(%6.2f %6.2f %6.2f) corrected=(%6.2f %6.2f %6.2f) acc=(%5.2f %5.2f %5.2f)\n",
                      (unsigned long)now, (unsigned long)sample_count,
                      imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z,
                      gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);
        sample_count = 0;
        rate_window_start = now;
      }
#endif
    }

    // Poll faster than the 500Hz data rate so no sample is missed; update()
    // returns 0 when nothing new has arrived.
    vTaskDelayUntil(&last_ticks, pdMS_TO_TICKS(1));
  }
}
