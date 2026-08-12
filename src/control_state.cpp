#include "control_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {

SemaphoreHandle_t lock = NULL;

bool enabled = false;
VelocityCommand_t velocity = {0.0f, 0.0f};
PidGains_t gains = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
bool gains_dirty = false;
ControlFeedback_t feedback = {0, 0, 0.0f, 0, false};

struct Guard {
  Guard() { if (lock != NULL) { xSemaphoreTake(lock, portMAX_DELAY); } }
  ~Guard() { if (lock != NULL) { xSemaphoreGive(lock); } }
};

}  // namespace

void ControlStateInit(const PidGains_t* initial_gains) {
  if (lock == NULL) {
    lock = xSemaphoreCreateMutex();
  }
  Guard g;
  if (initial_gains != NULL) {
    gains = *initial_gains;
  }
  gains_dirty = false;
  enabled = false;
  velocity.linear = 0.0f;
  velocity.angular = 0.0f;
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
