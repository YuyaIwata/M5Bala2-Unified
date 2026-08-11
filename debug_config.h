#pragma once

// Set to 1 to stream control-loop and IMU state over serial (115200 baud) for
// diagnosis. Leave at 0 for normal operation: the prints add I2C-adjacent work
// inside the control tasks.
#define BALA_DEBUG_TELEMETRY 0

// The 20Hz control-loop dump is ~95 bytes per line; at 115200 baud a burst can
// block the PID task long enough to distort the very timing being measured.
// Turn it off when the loop period itself is what is under investigation.
#define BALA_DEBUG_STREAM 0
