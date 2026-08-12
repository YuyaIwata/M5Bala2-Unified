#include "../debug_config.h"
#include "display.h"
#include <M5Unified.h>
#include "imu_filter.h"

namespace {

constexpr int16_t SCREEN_W = 320;
constexpr int16_t SCREEN_H = 240;

// A full 320x240x16bpp sprite is 150KB, which only fits because the Fire has
// PSRAM. Pushing it over SPI is the dominant cost, so the frame rate is capped
// rather than run as fast as possible.
constexpr uint32_t FRAME_PERIOD_MS = 50;  // 20 FPS; a full-screen push measures ~45ms

// Waveform geometry. One sample per horizontal pixel.
constexpr int16_t WAVE_X = 34;
constexpr int16_t WAVE_W = 280;
constexpr int16_t WAVE_BASE_Y = 150;
constexpr int16_t WAVE_SCALE = 3;   // pixels per degree
constexpr int16_t WAVE_CLAMP = 70;  // +-70px, keeps the trace inside its band

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

void DrawFrame(float angle) {
  canvas.fillScreen(TFT_BLACK);

  const uint16_t grid = canvas.color565(48, 48, 48);
  const uint16_t axis = canvas.color565(96, 96, 96);

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
  canvas.setTextColor(TFT_WHITE);
  canvas.setFont(&fonts::Font4);
  canvas.setCursor(8, 8);
  canvas.printf("%+6.2f deg", angle);

  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(axis);
  canvas.setCursor(8, WAVE_BASE_Y - WAVE_CLAMP - 1);
  canvas.printf("+%d", WAVE_CLAMP / WAVE_SCALE);
  canvas.setCursor(8, WAVE_BASE_Y + WAVE_CLAMP - 14);
  canvas.printf("-%d", WAVE_CLAMP / WAVE_SCALE);

  if (banner_calibration) {
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(TFT_YELLOW);
    canvas.setCursor(8, SCREEN_H - 24);
    canvas.print("calibration mode");
  }

  canvas.pushSprite(0, 0);
}

void DisplayTask(void *arg) {
  uint32_t last_ticks = xTaskGetTickCount();

  for (;;) {
    float angle = getAngle();
    PushSample(angle);
    DrawFrame(angle);

#if BALA_DEBUG_TELEMETRY
    static uint32_t frames = 0;
    static uint32_t window_start = 0;
    frames++;
    uint32_t now = millis();
    if (now - window_start >= 1000) {
      Serial.printf("[lcd] %lu fps (core %d)\n", (unsigned long)frames, xPortGetCoreID());
      frames = 0;
      window_start = now;
    }
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

  // Core 0 handles the UI (and, later, micro-ROS); core 1 stays with control.
  // Lowest priority: dropping frames is always preferable to delaying control.
  xTaskCreatePinnedToCore(DisplayTask, "display_task", 4 * 1024, NULL, 1, NULL, 0);
}
