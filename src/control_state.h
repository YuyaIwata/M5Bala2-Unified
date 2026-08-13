#ifndef __CONTROL_STATE_H__
#define __CONTROL_STATE_H__

#include "Arduino.h"

// Shared state between the ROS task on core 0 and the control task on core 1.
//
// Every accessor takes a short mutex. Individual floats would be atomic on this
// architecture, but a gain set or a velocity command has to be applied as a
// group: a control cycle that saw a new kp with the old kd, or linear.x from one
// message with angular.z from the next, would act on a state that was never
// requested.

typedef struct {
  float linear;   // forward/backward, -1.0 .. 1.0
  float angular;  // turn rate, -1.0 .. 1.0 (curves and spin turns)
} VelocityCommand_t;

typedef struct {
  float kp, ki, kd;          // angle PID
  float s_kp, s_ki, s_kd;    // speed PID
} PidGains_t;

// Drive scaling, tunable at runtime. Retuning these needs the robot balancing on
// the floor, which is exactly when reflashing is most disruptive.
typedef struct {
  float speed_scale;     // encoder counts per control cycle at linear = 1.0
  float yaw_rate_scale;  // deg/s of yaw commanded at angular = 1.0
  float yaw_kp;          // differential PWM per degree of heading error
  float yaw_kd;          // damping on the heading error
  float pos_kp;          // commanded wheel speed per encoder count of position error
  float pos_kd;          // velocity feedback damping on the position loop
} DriveScale_t;

typedef struct {
  int32_t encoder_left;
  int32_t encoder_right;
  float wheel_speed;      // filtered encoder delta per control cycle
  int16_t pwm_output;
  bool enabled;
  float yaw_measured;  // integrated gyro Z, degrees
  float yaw_target;    // integrated command, degrees
  int16_t turn_pwm;    // differential the heading loop asked for
} ControlFeedback_t;

void ControlStateInit(const PidGains_t* initial_gains, const DriveScale_t* initial_scale,
                      float initial_center_angle);

// Called from the ROS task (core 0).
void ControlSetEnabled(bool enabled);
void ControlSetVelocity(const VelocityCommand_t* cmd);
void ControlSetGains(const PidGains_t* gains);
void ControlSetDriveScale(const DriveScale_t* scale);

// The balance setpoint, in degrees. Trimming it is how the standing position is
// matched to where the centre of mass actually is; a setpoint that leans even a
// fraction of a degree makes the robot creep.
void ControlSetCenterAngle(float degrees);
void ControlRequestSaveCalibration();

// Called from the control task (core 1).
bool ControlIsEnabled();
void ControlGetVelocity(VelocityCommand_t* cmd);
void ControlGetDriveScale(DriveScale_t* scale);
float ControlGetCenterAngle();
// True once per request; the caller performs the NVS write.
bool ControlTakeSaveRequest();
// Returns true and clears the flag when new gains have arrived, so the control
// task only reconfigures the PID objects on change.
bool ControlTakeGains(PidGains_t* gains);
void ControlPublishFeedback(const ControlFeedback_t* feedback);

// Called from the ROS and display tasks (core 0).
void ControlGetFeedback(ControlFeedback_t* feedback);

#endif
