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

typedef struct {
  int32_t encoder_left;
  int32_t encoder_right;
  float wheel_speed;      // filtered encoder delta per control cycle
  int16_t pwm_output;
  bool enabled;
} ControlFeedback_t;

void ControlStateInit(const PidGains_t* initial_gains);

// Called from the ROS task (core 0).
void ControlSetEnabled(bool enabled);
void ControlSetVelocity(const VelocityCommand_t* cmd);
void ControlSetGains(const PidGains_t* gains);

// Called from the control task (core 1).
bool ControlIsEnabled();
void ControlGetVelocity(VelocityCommand_t* cmd);
// Returns true and clears the flag when new gains have arrived, so the control
// task only reconfigures the PID objects on change.
bool ControlTakeGains(PidGains_t* gains);
void ControlPublishFeedback(const ControlFeedback_t* feedback);

// Called from the ROS and display tasks (core 0).
void ControlGetFeedback(ControlFeedback_t* feedback);

#endif
