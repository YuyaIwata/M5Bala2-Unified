#include "debug_config.h"
#include <M5Unified.h>
#include "freertos/FreeRTOS.h"
#include "./src/imu_filter.h"
#include "./src/MadgwickAHRS.h"
#include "./src/bala.h"
#include "./src/pid.h"
#include "./src/calibration.h"
#include "./src/display.h"
#include "./src/control_state.h"
#include "./src/ros_interface.h"

static void PIDTask(void *arg);

static float angle_point = -1.5;

// The upstream gains were tuned against a control loop that, despite asking for
// 5ms, actually ran at ~11.9ms: its IMU task drained the MPU6886 FIFO in one
// burst over a 100kHz bus and held the I2C mutex for most of that time, which
// stalled this task. Measured on hardware: 11902us average while balancing.
//
// Polling the IMU through M5Unified removes that stall, so the loop now holds
// its intended 5ms. PID::Update takes a raw difference for the derivative and a
// raw sum for the integral, with no dt normalisation, so both terms scale with
// the period: at 5ms the derivative shrinks by 11.9/5.0 and the integral grows
// by the same factor. kd carries the damping here, and losing 2.4x of it is
// what let the robot accelerate into a fall instead of recovering.
//
// Rescale so the per-second behaviour matches the tuning it was built for.
// Rescaling the gains for a 5ms loop was not enough: measured against the
// original firmware, the same `out` produced 7-8x less wheel acceleration. The
// control output is computed correctly, so the commands are not reaching the
// Bala2 module. The loop also drives the module 2.4x more often than the
// original did (200Hz of read+write instead of 84Hz), which the module's own
// MCU may not keep up with.
//
// Run the loop at the period the original actually achieved, so both the gain
// scaling and the I2C command rate match the configuration that balances.
static const uint32_t PID_PERIOD_MS = 12;

// A cmd_vel of 1.0 shifts the speed PID setpoint by this many encoder counts
// per control cycle. Kept modest: the wheels have to stay available to the
// balance loop, and the wheel speed observed while balancing spans about +-17.
static const float DRIVE_SPEED_SCALE = 12.0f;

// Differential PWM applied at full angular command.
static const float TURN_PWM_SCALE = 250.0f;

float kp = 24.0f, ki = 0.0f, kd = 90.0f;
float s_kp = 15.0f, s_ki = 0.075f, s_kd = 0.0f;

bool calibration_mode = false;

Bala bala;

PID pid(angle_point, kp, ki, kd);
PID speed_pid(0, s_kp, s_ki, s_kd);

// the setup routine runs once when M5Stack starts up
void setup(){
  // Initialize the M5Stack object

  auto cfg = M5.config();
  cfg.internal_imu = true;
  cfg.internal_spk = false;
  cfg.internal_mic = false;
  M5.begin(cfg);

  Serial.begin(115200);

  // M5Unified latches button state in update(), so it must run once before the
  // boot-time button checks below.
  M5.update();

  float x_offset, y_offset, z_offset;
  float angle_center;
  calibrationInit();

  if (M5.BtnB.isPressed()) {
    calibrationGryo();
    calibration_mode = true;
  }

  if (M5.BtnC.isPressed()) {
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Charge mode");
    while (1) {
        if (M5.Power.isCharging() == m5::Power_Class::is_charging) {
            M5.Lcd.println("Start charging...");
            while(1) {
                // M5Unified has no isChargeFull(); the IP5306 reports a full
                // pack as 100% instead.
                if (M5.Power.getBatteryLevel() >= 100)
                    M5.Lcd.println("Charge completed!");
                delay(5000);
            }
        }
        delay(500);
    }
  }

  calibrationGet(&x_offset, &y_offset, &z_offset, &angle_center);
  Serial.printf("x: %.4f, y: %.4f, z: %.4f, angle: %.2f", x_offset, y_offset, z_offset, angle_center);

  angle_point = angle_center;
  pid.SetPoint(angle_point);

  SemaphoreHandle_t i2c_mutex;;
  i2c_mutex = xSemaphoreCreateMutex();
  bala.SetMutex(&i2c_mutex);   
  ImuTaskStart(x_offset, y_offset, z_offset, &i2c_mutex);
  PidGains_t initial_gains = {kp, ki, kd, s_kp, s_ki, s_kd};
  ControlStateInit(&initial_gains);

  xTaskCreatePinnedToCore(PIDTask, "pid_task", 4 * 1024, NULL, 4, NULL, 1);

  DisplayTaskStart(calibration_mode);
  RosTaskStart();
}

// Rendering lives in its own task on core 0, so this loop only polls the
// buttons. It stays on core 1 with the control tasks (ARDUINO_RUNNING_CORE=1)
// at the lowest priority, and the work is a few GPIO reads.
void loop() {
  vTaskDelay(pdMS_TO_TICKS(20));

  M5.update();
  if (M5.BtnA.wasPressed()) {
    angle_point += 0.25;
    pid.SetPoint(angle_point);
  }
  
  if (M5.BtnB.wasPressed()) {
    if (calibration_mode) {
      calibrationSaveCenterAngle(angle_point);
    }

  }

  if (M5.BtnC.wasPressed()) {
    angle_point -= 0.25;
    pid.SetPoint(angle_point);
  }
} 

static void PIDTask(void *arg) {
  float bala_angle;
  float motor_speed = 0;

  int16_t pwm_speed = 0;
  int16_t pwm_output = 0;
  int16_t pwm_angle = 0;

  int32_t encoder = 0;
  int32_t last_encoder = 0;
  uint32_t last_ticks = 0;

  VelocityCommand_t velocity = {0.0f, 0.0f};
  ControlFeedback_t feedback = {0, 0, 0.0f, 0, false};

  pid.SetOutputLimits(1023, -1023);
  pid.SetDirection(-1);
  
  speed_pid.SetIntegralLimits(40, -40);
  speed_pid.SetOutputLimits(1023, -1023);
  speed_pid.SetDirection(1);

  for(;;) {
    vTaskDelayUntil(&last_ticks, pdMS_TO_TICKS(PID_PERIOD_MS));

    // in imu task update, update freq is 200HZ
    bala_angle = getAngle();
    
    // Get motor encoder value
    bala.UpdateEncoder();

    encoder = bala.wheel_left_encoder + bala.wheel_right_encoder;
    // motor_speed filter
    motor_speed = 0.8 * motor_speed + 0.2 * (encoder - last_encoder);
    last_encoder = encoder;

    // Gains arrive from ROS on core 0; reconfigure only when they change.
    PidGains_t new_gains;
    if (ControlTakeGains(&new_gains)) {
      kp = new_gains.kp; ki = new_gains.ki; kd = new_gains.kd;
      s_kp = new_gains.s_kp; s_ki = new_gains.s_ki; s_kd = new_gains.s_kd;
      pid.UpdateParam(kp, ki, kd);
      speed_pid.UpdateParam(s_kp, s_ki, s_kd);
    }

    ControlGetVelocity(&velocity);
    bool enabled = ControlIsEnabled();

    if (enabled && fabs(bala_angle) < 70) {
      // The speed PID holds position at zero wheel speed, so a drive command is
      // expressed as an offset to its setpoint rather than added to the output.
      // That keeps the balance loop in charge of staying upright.
      speed_pid.SetPoint(velocity.linear * DRIVE_SPEED_SCALE);

      pwm_angle = (int16_t)pid.Update(bala_angle);
      pwm_speed = (int16_t)speed_pid.Update(motor_speed);
      pwm_output = pwm_speed + pwm_angle;
      if(pwm_output > 1023) { pwm_output = 1023; }
      if(pwm_output < -1023) { pwm_output = -1023; }

      // Differential term steers: positive angular turns one way, negative the
      // other. A spin turn is simply linear = 0 with angular at full scale.
      int16_t turn = (int16_t)(velocity.angular * TURN_PWM_SCALE);
      int16_t pwm_left = pwm_output - turn;
      int16_t pwm_right = pwm_output + turn;
      if(pwm_left > 1023) { pwm_left = 1023; }
      if(pwm_left < -1023) { pwm_left = -1023; }
      if(pwm_right > 1023) { pwm_right = 1023; }
      if(pwm_right < -1023) { pwm_right = -1023; }
      bala.SetSpeed(pwm_left, pwm_right);
    } else {
      pwm_angle = 0;
      pwm_speed = 0;
      pwm_output = 0;
      speed_pid.SetPoint(0);
      bala.SetSpeed(0, 0);
      bala.SetEncoder(0, 0);
      speed_pid.SetIntegral(0);
      last_encoder = 0;
      motor_speed = 0;
    }

    feedback.encoder_left = bala.wheel_left_encoder;
    feedback.encoder_right = bala.wheel_right_encoder;
    feedback.wheel_speed = motor_speed;
    feedback.pwm_output = pwm_output;
    feedback.enabled = enabled;
    ControlPublishFeedback(&feedback);

#if BALA_DEBUG_TELEMETRY
    // The control law is dominated by kd * (angle change per cycle), so both the
    // achieved loop period and the derivative input are worth watching.
    static uint32_t last_us = 0;
    static uint32_t period_min = 0xffffffff, period_max = 0, period_sum = 0, period_n = 0;
    static float last_angle = 0;
    static uint32_t next_stat_time = 0;

    uint32_t now_us = micros();
    if (last_us != 0) {
      uint32_t d = now_us - last_us;
      if (d < period_min) { period_min = d; }
      if (d > period_max) { period_max = d; }
      period_sum += d;
      period_n++;
    }
    last_us = now_us;

    float d_angle = bala_angle - last_angle;
    last_angle = bala_angle;

#if BALA_DEBUG_STREAM
    static uint32_t next_log_time = 0;
    if (millis() > next_log_time) {
      next_log_time = millis() + 50;
      Serial.printf("angle=%7.2f d=%+6.3f enc=%8ld speed=%8.2f pwm_angle=%5d pwm_speed=%5d out=%5d\n",
                    bala_angle, d_angle, (long)encoder, motor_speed, pwm_angle, pwm_speed, pwm_output);
    }
#else
    (void)d_angle;
#endif

    if (millis() > next_stat_time && period_n > 0) {
      next_stat_time = millis() + 1000;
      Serial.printf("[pid] period avg=%luus min=%luus max=%luus n=%lu\n",
                    (unsigned long)(period_sum / period_n), (unsigned long)period_min,
                    (unsigned long)period_max, (unsigned long)period_n);
      period_min = 0xffffffff; period_max = 0; period_sum = 0; period_n = 0;
    }
#endif
  }
}

