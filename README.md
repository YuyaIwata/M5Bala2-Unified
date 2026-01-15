# Bala2/Bala2-Fire

## Overview

### SKU:K014-C/K014-E

Bala2/Bala2-Fire is a balancing car application. This product is a self-balancing robot composed of the M5Stack Gray/Fire and the BALA2 motor base. The base uses the STM32F030C8T6 as the main controller, powered by two N20 encoded reduction motors, and has a built-in 1200mAh battery. The name "BALA" comes from the abbreviation of "Balance," and this is the second-generation product. The BALA2 base includes a rich set of interfaces, supporting 8 servo motors in addition to the regular PortB and PortC, with 4 interfaces directly connectable and the other 4 needing to be extended from inside the base. You can program it to move freely or develop remote control functions with WiFi. Even if you have never worked with balancing car programs before, you can quickly complete programming and control it using UiFlow. The product comes pre-installed with a balancing car application, using a PID closed-loop algorithm to maintain vertical balance during operation, and utilizes accelerometer and gyroscope attitude data to correct its direction and position.

## Related Link

- [Document & Datasheet of Bala2](https://docs.m5stack.com/en/app/bala2)
- [Document & Datasheet of Bala2-Fire](https://docs.m5stack.com/en/app/bala2fire)

## Required Libraries:

- [M5Stack](https://github.com/m5stack/m5stack)

## Notes:

1. If you encountered error "fatal error: rom/miniz.h: No such file or directory", please downgrade your M5Stack board manager to version above 2.1.4.

## License

- [Bala2/Bala2-Fire - MIT](LICENSE)
