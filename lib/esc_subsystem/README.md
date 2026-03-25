# easy-esc-esp32 library

Beginner-friendly ESC control for ESP32.

## Include

```cpp
#include <easy_esc.h>
```

## Main classes

1. `esc::EasyEscMotor` (single motor, servo-style)
2. `esc::EasyEsc` (multi-motor helper)

## Quick single-motor example

```cpp
#include <easy_esc.h>

esc::EasyEscMotor MOTOR1(GPIO_NUM_41, GPIO_NUM_3, DSHOT300);

void setup() {
  MOTOR1.begin();
  MOTOR1.arm();
}

void loop() {
  MOTOR1.update();
  MOTOR1.spinPercent(20);
}
```

## Defaults

- DShot mode: `DSHOT300`
- raw throttle range: `0` or `48..2047`

## Time units

- timeout and refresh settings: milliseconds
- passthrough timing settings: microseconds

## Passthrough status

Passthrough is experimental:

- BLHeli_S bootloader flow is incomplete
- CRC/framing checks are incomplete
- read/write/erase behavior is scaffold-level
