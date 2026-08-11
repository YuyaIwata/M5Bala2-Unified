# Bala2/Bala2-Fire

> **Fork notice**
> This is a fork of [m5stack/M5Bala2](https://github.com/m5stack/M5Bala2). The device layer has been migrated from the `M5Stack` library to M5Unified so that it builds with ESP32 core 3.x (ESP-IDF 5.x). Verified on real hardware (M5Stack Fire + BALA2) to balance as stably as upstream.
>
> Detailed documentation, including the migration notes and why the control loop period is pinned to 12ms, is in **[README_ja.md](README_ja.md)** (Japanese).

## Overview

### SKU:K014-C/K014-E

Bala2/Bala2-Fire is a balancing car application. This product is a self-balancing robot composed of the M5Stack Gray/Fire and the BALA2 motor base. The base uses the STM32F030C8T6 as the main controller, powered by two N20 encoded reduction motors, and has a built-in 1200mAh battery. The name "BALA" comes from the abbreviation of "Balance," and this is the second-generation product. The BALA2 base includes a rich set of interfaces, supporting 8 servo motors in addition to the regular PortB and PortC, with 4 interfaces directly connectable and the other 4 needing to be extended from inside the base. You can program it to move freely or develop remote control functions with WiFi. Even if you have never worked with balancing car programs before, you can quickly complete programming and control it using UiFlow. The product comes pre-installed with a balancing car application, using a PID closed-loop algorithm to maintain vertical balance during operation, and utilizes accelerometer and gyroscope attitude data to correct its direction and position.

## Related Link

- [Document & Datasheet of Bala2](https://docs.m5stack.com/en/app/bala2)
- [Document & Datasheet of Bala2-Fire](https://docs.m5stack.com/en/app/bala2fire)

## Required Libraries:

- [M5Unified](https://github.com/m5stack/M5Unified)
- [M5GFX](https://github.com/m5stack/M5GFX)

Both libraries and the board package are pinned in [sketch.yaml](sketch.yaml) and are installed automatically on the first build, so no manual board manager setup is required.

```bash
arduino-cli compile --profile m5stack_fire .
arduino-cli upload --profile m5stack_fire -p /dev/cu.usbserial-XXXXXXXX .
```

## Notes:

1. Upstream requires the M5Stack board manager to stay at 2.1.4, because the `M5Stack` library depends on `rom/miniz.h`, which was removed in ESP-IDF 5.x. This fork removes that constraint by moving to M5Unified: it builds against `m5stack:esp32@3.3.8` (ESP-IDF v5.5.4).
2. Gyro offsets are now stored in deg/s under new NVS keys, so values saved by the upstream firmware are not read. Hold button B while powering on to re-run the calibration.

## License

- [Bala2/Bala2-Fire - MIT](LICENSE)
