#include "../debug_config.h"
#include "display.h"
#include <M5Unified.h>
#include "imu_filter.h"
#include "control_state.h"
#include "ros_interface.h"

namespace {

constexpr int16_t SCREEN_W = 320;
constexpr int16_t SCREEN_H = 240;

// A full 320x240x16bpp sprite is 150KB, which only fits because the Fire has
// PSRAM. Pushing it over SPI is the dominant cost, so the frame rate is capped
// rather than run as fast as possible.
constexpr uint32_t FRAME_PERIOD_MS = 100;  // 10 FPS: core 0 also carries micro-ROS and WiFi

// Waveform geometry. One sample per horizontal pixel.
constexpr int16_t WAVE_X = 34;
constexpr int16_t WAVE_W = 280;
constexpr int16_t WAVE_BASE_Y = 120;
constexpr int16_t WAVE_SCALE = 3;   // pixels per degree
constexpr int16_t WAVE_CLAMP = 60;  // +-60px, keeps the trace inside its band

constexpr int16_t STATUS_Y = 196;

M5Canvas canvas(&M5.Display);
int16_t wave[WAVE_W];
size_t wave_head = 0;
bool banner_calibration = false;

void PushSample(float angle) {
  int16_t v = (int16_t)(angle * WAVE_SCALE);
  if (v > WAVE_CLAMP) { v = WAVE_CLAMP; }
  if (v < -WAVE_CLAMP) { v = -WAVE_CLAMP; }
  wave[wave_head] = v;
  wave_head = (wave_head + 1) % WAVE_W;
}

const char* LinkLabel(RosLinkState_t state) {
  switch (state) {
    case ROS_LINK_DISABLED:   return "ROS off";
    case ROS_LINK_WIFI_WAIT:  return "WiFi...";
    case ROS_LINK_AGENT_WAIT: return "Agent...";
    case ROS_LINK_CONNECTED:  return "ROS ok";
  }
  return "?";
}

uint16_t LinkColor(RosLinkState_t state) {
  switch (state) {
    case ROS_LINK_CONNECTED: return TFT_GREEN;
    case ROS_LINK_DISABLED:  return TFT_DARKGREY;
    default:                 return TFT_ORANGE;
  }
}

void DrawStatusRow(const ControlFeedback_t& fb, const VelocityCommand_t& cmd) {
  const uint16_t dim = canvas.color565(96, 96, 96);

  canvas.setFont(&fonts::Font4);
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setTextColor(fb.enabled ? TFT_GREEN : TFT_RED);
  canvas.setCursor(8, STATUS_Y);
  canvas.print(fb.enabled ? "RUN " : "STOP");

  // Direction reads from the command rather than the wheels: at rest the balance
  // loop is constantly reversing the wheels, so wheel sign is not the intent.
  const char* drive = "----";
  uint16_t drive_color = dim;
  if (cmd.linear > 0.05f) { drive = "FWD "; drive_color = TFT_CYAN; }
  else if (cmd.linear < -0.05f) { drive = "REV "; drive_color = TFT_CYAN; }

  // REP-103: a positive angular.z is counter-clockwise seen from above, which is
  // a left turn. The differential was flipped to match that convention; this
  // label was not, so the display contradicted the robot.
  const char* turn = "  ";
  if (cmd.angular > 0.05f) { turn = " L"; }
  else if (cmd.angular < -0.05f) { turn = " R"; }

  canvas.setTextColor(drive_color);
  canvas.setCursor(96, STATUS_Y);
  canvas.print(drive);
  canvas.setTextColor(cmd.angular > 0.05f || cmd.angular < -0.05f ? TFT_CYAN : dim);
  canvas.print(turn);

  // Command magnitude bar, signed around the centre.
  constexpr int16_t BAR_X = 190;
  constexpr int16_t BAR_W = 120;
  constexpr int16_t BAR_H = 10;
  const int16_t bar_y = STATUS_Y + 30;
  const int16_t centre = BAR_X + BAR_W / 2;
  canvas.drawRect(BAR_X, bar_y, BAR_W, BAR_H, dim);
  canvas.drawFastVLine(centre, bar_y, BAR_H, dim);
  int16_t len = (int16_t)(cmd.linear * (BAR_W / 2 - 2));
  if (len > 0) {
    canvas.fillRect(centre + 1, bar_y + 2, len, BAR_H - 4, TFT_CYAN);
  } else if (len < 0) {
    canvas.fillRect(centre + len, bar_y + 2, -len, BAR_H - 4, TFT_CYAN);
  }
}

void DrawFrame(float angle, bool push) {
  canvas.fillScreen(TFT_BLACK);

  const uint16_t grid = canvas.color565(48, 48, 48);
  const uint16_t axis = canvas.color565(96, 96, 96);

  ControlFeedback_t fb;
  ControlGetFeedback(&fb);
  VelocityCommand_t cmd;
  ControlGetVelocity(&cmd);
  RosLinkState_t link = RosGetLinkState();

  // Header: angle on the left, link state on the right.
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setTextColor(TFT_WHITE);
  canvas.setFont(&fonts::Font4);
  canvas.setCursor(8, 6);
  canvas.printf("%+6.2f deg", angle);

  canvas.setFont(&fonts::Font2);
  canvas.setTextDatum(textdatum_t::top_right);
  canvas.setTextColor(LinkColor(link));
  canvas.drawString(LinkLabel(link), SCREEN_W - 8, 10);

  // A balancing robot that cannot recover looks like a tuning problem and is
  // often just a flat battery, so the charge state stays on screen. Note this is
  // the M5Stack's own cell; the BALA2 base powers the motors from its own pack,
  // which cannot be read from here.
  // The Fire's IP5306 has no voltage register, so getBatteryVoltage() returns 0
  // and the level only moves in 25% steps. Show the level alone.
  int32_t level = M5.Power.getBatteryLevel();
  uint16_t batt_color = TFT_GREEN;
  if (level < 30) { batt_color = TFT_RED; }
  else if (level < 60) { batt_color = TFT_ORANGE; }
  canvas.setTextColor(batt_color);
  canvas.drawString("BAT " + String(level) + "%", SCREEN_W - 8, 28);

  // Horizontal guides every 5 degrees, with the zero line emphasised.
  for (int16_t deg = -20; deg <= 20; deg += 5) {
    int16_t y = WAVE_BASE_Y - deg * WAVE_SCALE;
    if (y < WAVE_BASE_Y - WAVE_CLAMP || y > WAVE_BASE_Y + WAVE_CLAMP) { continue; }
    canvas.drawFastHLine(WAVE_X, y, WAVE_W, deg == 0 ? axis : grid);
  }

  // The waveform scrolls left to right; wave_head is the oldest sample.
  for (int16_t i = 1; i < WAVE_W; i++) {
    size_t a = (wave_head + i - 1) % WAVE_W;
    size_t b = (wave_head + i) % WAVE_W;
    canvas.drawLine(WAVE_X + i - 1, WAVE_BASE_Y - wave[a],
                    WAVE_X + i, WAVE_BASE_Y - wave[b], TFT_GREEN);
  }

  canvas.drawRect(WAVE_X - 1, WAVE_BASE_Y - WAVE_CLAMP - 1,
                  WAVE_W + 2, WAVE_CLAMP * 2 + 2, grid);

  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(axis);
  canvas.setCursor(8, WAVE_BASE_Y - WAVE_CLAMP - 1);
  canvas.printf("+%d", WAVE_CLAMP / WAVE_SCALE);
  canvas.setCursor(8, WAVE_BASE_Y + WAVE_CLAMP - 14);
  canvas.printf("-%d", WAVE_CLAMP / WAVE_SCALE);

  DrawStatusRow(fb, cmd);

  if (banner_calibration) {
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(TFT_YELLOW);
    canvas.setTextDatum(textdatum_t::top_right);
    canvas.drawString("calibration mode", SCREEN_W - 8, SCREEN_H - 18);
  }

  if (push) { canvas.pushSprite(0, 0); }
}

void DisplayTask(void *arg) {
  uint32_t last_ticks = xTaskGetTickCount();

  for (;;) {
    float angle = getAngle();
    PushSample(angle);

#if BALA_DEBUG_TELEMETRY
    // Split the frame cost: the sprite lives in PSRAM, so both the drawing into
    // it and the SPI push out of it are candidates for being the bottleneck.
    uint32_t t0 = micros();
    DrawFrame(angle, false);
    uint32_t t1 = micros();
    canvas.pushSprite(0, 0);
    uint32_t t2 = micros();

    static uint32_t frames = 0, draw_us = 0, push_us = 0, window_start = 0;
    frames++;
    draw_us += t1 - t0;
    push_us += t2 - t1;
    uint32_t now = millis();
    if (now - window_start >= 1000) {
      Serial.printf("[lcd] %lu fps draw=%luus push=%luus (core %d)\n",
                    (unsigned long)frames, (unsigned long)(draw_us / frames),
                    (unsigned long)(push_us / frames), xPortGetCoreID());
      frames = 0; draw_us = 0; push_us = 0; window_start = now;
    }
#else
    DrawFrame(angle, true);
#endif

    vTaskDelayUntil(&last_ticks, pdMS_TO_TICKS(FRAME_PERIOD_MS));
  }
}

}  // namespace

void DisplayTaskStart(bool calibration_mode) {
  banner_calibration = calibration_mode;

  // 16bpp keeps pushSprite a straight DMA-able copy; at 150KB it has to live in
  // PSRAM. Fall back to 8bpp if that allocation ever fails so the robot still
  // shows something.
  canvas.setPsram(true);
  canvas.setColorDepth(16);
  if (!canvas.createSprite(SCREEN_W, SCREEN_H)) {
    canvas.setColorDepth(8);
    if (!canvas.createSprite(SCREEN_W, SCREEN_H)) {
      M5.Display.fillScreen(TFT_BLACK);
      M5.Display.setCursor(0, 0);
      M5.Display.print("canvas alloc failed");
      return;
    }
  }

  for (int16_t i = 0; i < WAVE_W; i++) { wave[i] = 0; }

  // Core 0 handles the UI and micro-ROS; core 1 stays with control.
  // Lowest priority: dropping frames is always preferable to delaying control.
  xTaskCreatePinnedToCore(DisplayTask, "display_task", 4 * 1024, NULL, 1, NULL, 0);
}
