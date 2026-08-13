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
static const float DRIVE_SPEED_SCALE_DEFAULT = 25.0f;

// Yaw is held as an angle rather than a rate. A rate loop lets heading error
// accumulate, so driving straight would slowly curve; integrating the command
// into a target heading makes angular.z = 0 mean "hold the current heading" and
// a non-zero command mean "keep turning", which is the spin turn.
static const float YAW_RATE_SCALE_DEFAULT = 90.0f;  // deg/s at angular = 1.0
// Measured on hardware: at kp=8 the heading drifted 40 degrees before the loop
// caught up and the differential saturated. At kp=25 the same straight run held
// heading to 0.36 degrees. kd is raised with it to keep the response damped.
static const float YAW_KP_DEFAULT = 25.0f;          // differential PWM per degree
static const float YAW_KD_DEFAULT = 150.0f;

// Fore-aft position, in the same cascade shape as the heading loop: the outer
// loop turns a position error into the wheel speed the inner speed PID chases.
// Holding position is what makes a spin turn stay in place, and it also removes
// the slow creep the speed-only loop always had.
// Measured on hardware: 0.02 held position but oscillated 1170 counts, 0.005 was
// smoother yet drifted 1028 counts one way, 0.01 gave the best of both.
static const float POS_KP_DEFAULT = 0.01f;      // wheel speed per encoder count
// Left at zero deliberately. Damping the position loop with the wheel speed looked
// right on paper, but the inner speed PID already closes a loop on that same
// quantity: measured on hardware, pos_kd=0.4 grew the position swing from 812 to
// 3324 counts. The velocity feedback belongs to the inner loop only.
static const float POS_KD_DEFAULT = 0.0f;
static const float POS_SPEED_LIMIT = 30.0f;     // counts per control cycle
static const float POS_ERROR_LIMIT = 1500.0f;   // anti-windup on the target

// The steering term must never take the whole actuator away from balancing.
static const int16_t TURN_PWM_LIMIT = 500;

float kp = 24.0f, ki = 0.0f, kd = 90.0f;
// s_kd measured on hardware: 0 gave a 992 count position swing, 20 gave 854, 50
// gave 812 with the swing centred, 100 regressed to 860 and pushed the PWM demand
// from 336 to 548 as the derivative started amplifying noise.
float s_kp = 15.0f, s_ki = 0.075f, s_kd = 50.0f;

bool calibration_mode = false;

Bala bala;

PID pid(angle_point, kp, ki, kd);
PID speed_pid(0, s_kp, s_ki, s_kd);
PID yaw_pid(0, YAW_KP_DEFAULT, 0.0f, YAW_KD_DEFAULT);

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
  DriveScale_t initial_scale = {DRIVE_SPEED_SCALE_DEFAULT, YAW_RATE_SCALE_DEFAULT,
                                YAW_KP_DEFAULT, YAW_KD_DEFAULT, POS_KP_DEFAULT,
                                POS_KD_DEFAULT};
  ControlStateInit(&initial_gains, &initial_scale, angle_center);

  xTaskCreatePinnedToCore(PIDTask, "pid_task", 4 * 1024, NULL, 4, NULL, 1);

  DisplayTaskStart(calibration_mode);
  RosTaskStart();

  // Trimming the balance setpoint only makes sense while the robot is actually
  // balancing, so calibration mode starts the controller instead of waiting for
  // an /enable message. Hold button B at power-on to get here deliberately.
  if (calibration_mode) {
    ControlSetEnabled(true);
  }
}

// Rendering lives in its own task on core 0, so this loop only polls the
// buttons. It stays on core 1 with the control tasks (ARDUINO_RUNNING_CORE=1)
// at the lowest priority, and the work is a few GPIO reads.
void loop() {
  vTaskDelay(pdMS_TO_TICKS(20));

  M5.update();
  // Buttons and ROS both trim the same setpoint, so they go through the shared
  // state rather than writing pid directly.
  if (M5.BtnA.wasPressed()) {
    ControlSetCenterAngle(ControlGetCenterAngle() + 0.25f);
  }

  if (M5.BtnB.wasPressed()) {
    ControlRequestSaveCalibration();
  }

  if (M5.BtnC.wasPressed()) {
    ControlSetCenterAngle(ControlGetCenterAngle() - 0.25f);
  }

  // Preferences is opened on this core; keep the NVS write here rather than in
  // the ROS task.
  if (ControlTakeSaveRequest()) {
    float saved = ControlGetCenterAngle();
    calibrationSaveCenterAngle(saved);
    Serial.printf("center angle saved: %.3f\n", saved);
  }
} 

static void PIDTask(void *arg) {
  float bala_angle;
  float motor_speed = 0;

  int16_t pwm_speed = 0;
  int16_t pwm_output = 0;
  int16_t pwm_angle = 0;
  int16_t turn = 0;

  int32_t encoder = 0;
  int32_t last_encoder = 0;
  uint32_t last_ticks = 0;

  VelocityCommand_t velocity = {0.0f, 0.0f};
  DriveScale_t scale = {DRIVE_SPEED_SCALE_DEFAULT, YAW_RATE_SCALE_DEFAULT,
                        YAW_KP_DEFAULT, YAW_KD_DEFAULT, POS_KP_DEFAULT,
                        POS_KD_DEFAULT};
  float position_target = 0.0f;
  ImuState_t imu_state;
  float last_linear = 0.0f;
  bool last_enabled = false;
  float yaw_measured = 0.0f;   // integrated gyro Z, degrees, relative to startup
  float yaw_target = 0.0f;
  const float dt = PID_PERIOD_MS * 0.001f;
  ControlFeedback_t feedback = {0, 0, 0.0f, 0, false, 0.0f, 0.0f, 0};

  pid.SetOutputLimits(1023, -1023);
  pid.SetDirection(-1);
  
  speed_pid.SetIntegralLimits(40, -40);
  speed_pid.SetOutputLimits(1023, -1023);
  speed_pid.SetDirection(1);

  yaw_pid.SetOutputLimits(TURN_PWM_LIMIT, -TURN_PWM_LIMIT);
  yaw_pid.SetDirection(1);

  for(;;) {
    vTaskDelayUntil(&last_ticks, pdMS_TO_TICKS(PID_PERIOD_MS));

    getImuState(&imu_state);
    bala_angle = imu_state.angle;

    // The MPU6886 Z axis points up in the balance pose (the accelerometer reads
    // +1g there), so gyro Z is the yaw rate with the right-hand sign REP-103 uses.
    yaw_measured += imu_state.gyro_z * dt;
    
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
    ControlGetDriveScale(&scale);
    angle_point = ControlGetCenterAngle();
    pid.SetPoint(angle_point);
    bool enabled = ControlIsEnabled();

    // Enabling while the robot is held well away from balance left the heading
    // loop with a large error on its first cycle and it asked for the full
    // differential straight away. Re-zero both outer loops on the transition.
    if (enabled && !last_enabled) {
      yaw_target = yaw_measured;
      position_target = (bala.wheel_left_encoder + bala.wheel_right_encoder) * 0.5f;
      speed_pid.SetIntegral(0);
      turn = 0;
    }
    last_enabled = enabled;

    if (enabled && fabs(bala_angle) < 70) {
      // The speed PID holds position at zero wheel speed, so a drive command is
      // expressed as an offset to its setpoint rather than added to the output.
      // That keeps the balance loop in charge of staying upright.
      // A drive command that ends leaves the speed integral holding the lean it
      // built up, which kept the robot rolling after cmd_vel returned to zero.
      // Clear it on the transition back to stop.
      if (fabs(velocity.linear) < 0.01f && fabs(last_linear) >= 0.01f) {
        speed_pid.SetIntegral(0);
      }
      last_linear = velocity.linear;

      // linear.x moves the target rather than the wheels, so releasing the stick
      // leaves a position to hold instead of a velocity to coast at.
      float position = (bala.wheel_left_encoder + bala.wheel_right_encoder) * 0.5f;
      position_target += velocity.linear * scale.speed_scale;
      float pos_error = position_target - position;
      // Without this the target runs away whenever the robot cannot keep up, and
      // the accumulated error comes back as a lurch when it finally can.
      if (pos_error > POS_ERROR_LIMIT) {
        position_target = position + POS_ERROR_LIMIT;
        pos_error = POS_ERROR_LIMIT;
      } else if (pos_error < -POS_ERROR_LIMIT) {
        position_target = position - POS_ERROR_LIMIT;
        pos_error = -POS_ERROR_LIMIT;
      }
      // motor_speed is the already filtered wheel velocity, which is exactly the
      // derivative of the position being controlled.
      float desired_speed = pos_error * scale.pos_kp - motor_speed * scale.pos_kd;
      if (desired_speed > POS_SPEED_LIMIT) { desired_speed = POS_SPEED_LIMIT; }
      if (desired_speed < -POS_SPEED_LIMIT) { desired_speed = -POS_SPEED_LIMIT; }
      speed_pid.SetPoint(desired_speed);

      pwm_angle = (int16_t)pid.Update(bala_angle);
      pwm_speed = (int16_t)speed_pid.Update(motor_speed);
      pwm_output = pwm_speed + pwm_angle;
      if(pwm_output > 1023) { pwm_output = 1023; }
      if(pwm_output < -1023) { pwm_output = -1023; }

      // Heading control. The target integrates the command, so angular.z = 0
      // holds the heading the robot already had and driving stays straight.
      yaw_target += velocity.angular * scale.yaw_rate_scale * dt;
      yaw_pid.UpdateParam(scale.yaw_kp, 0.0f, scale.yaw_kd);
      yaw_pid.SetPoint(yaw_target);
      turn = (int16_t)yaw_pid.Update(yaw_measured);
      int16_t pwm_left = pwm_output + turn;
      int16_t pwm_right = pwm_output - turn;
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
      // Track the heading and position while stopped so that enabling control
      // starts from zero error instead of wherever the robot was moved to.
      yaw_target = yaw_measured;
      position_target = (bala.wheel_left_encoder + bala.wheel_right_encoder) * 0.5f;
      bala.SetSpeed(0, 0);
      bala.SetEncoder(0, 0);
      speed_pid.SetIntegral(0);
      last_encoder = 0;
      motor_speed = 0;
      turn = 0;
    }

    feedback.encoder_left = bala.wheel_left_encoder;
    feedback.encoder_right = bala.wheel_right_encoder;
    feedback.wheel_speed = motor_speed;
    feedback.pwm_output = pwm_output;
    feedback.enabled = enabled;
    feedback.yaw_measured = yaw_measured;
    feedback.yaw_target = yaw_target;
    feedback.turn_pwm = turn;
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

