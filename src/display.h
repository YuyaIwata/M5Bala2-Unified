#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include "Arduino.h"

// Starts the rendering task. It is pinned to core 0 so that drawing never
// competes with the attitude control tasks on core 1: a full-screen sprite push
// takes tens of milliseconds, which would otherwise stretch the 12ms control
// period. The task only reads getAngle() and touches the display bus, never I2C.
void DisplayTaskStart(bool calibration_mode);

#endif
