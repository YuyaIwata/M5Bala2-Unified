#include "control_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {

SemaphoreHandle_t lock = NULL;

bool enabled = false;
VelocityCommand_t velocity = {0.0f, 0.0f};
PidGains_t gains = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
bool gains_dirty = false;
uint32_t velocity_stamp_ms = 0;
constexpr uint32_t VELOCITY_TIMEOUT_MS = 500;
ControlFeedback_t feedback = {0, 0, 0.0f, 0, false, 0.0f, 0.0f, 0};
DriveScale_t drive_scale = {25.0f, 90.0f, 25.0f, 150.0f, 0.01f, 0.0f};
float center_angle = 0.0f;
bool save_requested = false;

struct Guard {
  Guard() { if (lock != NULL) { xSemaphoreTake(lock, portMAX_DELAY); } }
  ~Guard() { if (lock != NULL) { xSemaphoreGive(lock); } }
};

}  // namespace

void ControlStateInit(const PidGains_t* initial_gains, const DriveScale_t* initial_scale,
                      float initial_center_angle) {
  if (lock == NULL) {
    lock = xSemaphoreCreateMutex();
  }
  Guard g;
  if (initial_gains != NULL) {
    gains = *initial_gains;
  }
  if (initial_scale != NULL) {
    drive_scale = *initial_scale;
  }
  center_angle = initial_center_angle;
  save_requested = false;
  gains_dirty = false;
  enabled = false;
  velocity.linear = 0.0f;
  velocity.angular = 0.0f;
  velocity_stamp_ms = 0;
}

void ControlSetEnabled(bool value) {
  Guard g;
  enabled = value;
  if (!value) {
    // Stopping must not leave a stale command that would take effect the moment
    // control is re-enabled.
    velocity.linear = 0.0f;
    velocity.angular = 0.0f;
  }
}

void ControlSetVelocity(const VelocityCommand_t* cmd) {
  if (cmd == NULL) { return; }
  Guard g;
  velocity = *cmd;
  velocity_stamp_ms = millis();
}

void ControlSetDriveScale(const DriveScale_t* scale) {
  if (scale == NULL) { return; }
  Guard g;
  drive_scale = *scale;
}

void ControlGetDriveScale(DriveScale_t* out) {
  if (out == NULL) { return; }
  Guard g;
  *out = drive_scale;
}

void ControlSetCenterAngle(float degrees) {
  Guard g;
  center_angle = degrees;
}

float ControlGetCenterAngle() {
  Guard g;
  return center_angle;
}

void ControlRequestSaveCalibration() {
  Guard g;
  save_requested = true;
}

bool ControlTakeSaveRequest() {
  Guard g;
  if (!save_requested) { return false; }
  save_requested = false;
  return true;
}

void ControlSetGains(const PidGains_t* new_gains) {
  if (new_gains == NULL) { return; }
  Guard g;
  gains = *new_gains;
  gains_dirty = true;
}

bool ControlIsEnabled() {
  Guard g;
  return enabled;
}

void ControlGetVelocity(VelocityCommand_t* cmd) {
  if (cmd == NULL) { return; }
  Guard g;
  // A drive command that stops arriving must not keep the robot going. Only the
  // velocity expires: dropping out of balance mode would put it on the floor,
  // which is worse than coasting to a stop.
  if (velocity_stamp_ms != 0 &&
      (millis() - velocity_stamp_ms) > VELOCITY_TIMEOUT_MS) {
    velocity.linear = 0.0f;
    velocity.angular = 0.0f;
  }
  *cmd = velocity;
}

bool ControlTakeGains(PidGains_t* out) {
  if (out == NULL) { return false; }
  Guard g;
  if (!gains_dirty) { return false; }
  *out = gains;
  gains_dirty = false;
  return true;
}

void ControlPublishFeedback(const ControlFeedback_t* value) {
  if (value == NULL) { return; }
  Guard g;
  feedback = *value;
}

void ControlGetFeedback(ControlFeedback_t* out) {
  if (out == NULL) { return; }
  Guard g;
  *out = feedback;
}
