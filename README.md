# easy-esc-esp32

Beginner-friendly ESP32 ESC library for capstone and lab projects.

This repo gives you a simple C++ object API to control BLHeli_S/Bluejay-compatible ESCs with DShot, without full flight-controller code.

## What This Project Includes

- `EasyEscMotor` class for single motor control
- `EasyEsc` class for multi-motor control (up to 4)
- DShot output using ESP32 RMT
- Arm/disarm safety controls
- Timeout and refresh controls
- Optional current monitor reading
- Experimental ESC passthrough scaffold

## Tested Target Profile

- ESC hardware: Flycolor Raptor BLS-04 4-in-1
- ESC firmware: BLHeli_S / Bluejay style
- Default DShot mode in this project: `DSHOT300`

## Folder Map

- [`src/main.cpp`](src/main.cpp): simple potentiometer-to-throttle example
- [`lib/esc_subsystem/src/easy_esc.h`](lib/esc_subsystem/src/easy_esc.h): beginner API header
- [`lib/esc_subsystem/src/easy_esc.cpp`](lib/esc_subsystem/src/easy_esc.cpp): implementation
- [`tools/esc_serial_ui.py`](tools/esc_serial_ui.py): desktop serial UI tool

## Important Concept: Macro vs Object

You do not need a special macro to use this library.

Use a C++ object, like Servo:

```cpp
esc::EasyEscMotor MOTOR1(GPIO_NUM_41, GPIO_NUM_3, DSHOT300, 20000, 10, 0.0f, 33.0f, true);
```

If you want cleaner configuration, define constants at the top of your file:

```cpp
constexpr gpio_num_t kMotorPin = GPIO_NUM_41;
constexpr gpio_num_t kCurrentPin = GPIO_NUM_3;
constexpr dshot_mode_t kMode = DSHOT300;
```

## Quick Start (Single Motor)

1. Remove propellers.
2. Wire ESC signal pin and ground correctly.
3. Build/upload with PlatformIO.
4. Open serial monitor at `115200`.
5. Follow arm and throttle flow.

### Minimal Code Pattern

```cpp
#include <Arduino.h>
#include <easy_esc.h>

esc::EasyEscMotor MOTOR1(GPIO_NUM_41, GPIO_NUM_3, DSHOT300, 20000, 10, 0.0f, 33.0f, true);

void setup() {
  Serial.begin(115200);

  if (!MOTOR1.begin()) {
    Serial.printf("begin failed, status=%u\n", (unsigned)MOTOR1.lastStatus());
    while (true) { delay(1000); }
  }

  MOTOR1.setTimeoutMs(20000);               // ms
  MOTOR1.setRefreshMs(10);                  // ms
  MOTOR1.setHoldArmOnSignalTimeout(true);   // keep armed, send 0 on timeout

  if (!MOTOR1.arm()) {
    Serial.printf("arm failed, status=%u\n", (unsigned)MOTOR1.lastStatus());
    while (true) { delay(1000); }
  }
}

void loop() {
  MOTOR1.update();          // required: call every loop
  MOTOR1.spinRaw(600);      // 0 or 48..2047
  delay(10);
}
```

## How To Make Objects

### Single motor object

```cpp
esc::EasyEscMotor MOTOR1(
  GPIO_NUM_41,  // motor signal pin
  GPIO_NUM_3,   // current monitor pin, or GPIO_NUM_NC to disable
  DSHOT300,     // DShot mode
  20000,        // timeout ms
  10,           // refresh ms
  0.0f,         // current zero offset mV
  33.0f,        // current scale mV per amp
  true          // reversed direction
);
```

### Multi motor object

```cpp
esc::EasyEsc ESC4(
  4,            // motor count
  GPIO_NUM_41,  // M1
  GPIO_NUM_42,  // M2
  GPIO_NUM_40,  // M3
  GPIO_NUM_39,  // M4
  GPIO_NUM_3,   // current pin
  DSHOT300,     // mode
  20000,        // timeout ms
  10,           // refresh ms
  0.0f,         // current zero offset mV
  33.0f,        // current scale mV per amp
  false,        // M1 reversed
  true,         // M2 reversed
  false,        // M3 reversed
  false         // M4 reversed
);
```

Use:

```cpp
ESC4.begin();
ESC4.arm();
ESC4.setMotorRaw(0, 600);   // motor index 0..3
ESC4.setAllRaw(500);
ESC4.update();
```

## API Cheat Sheet

- Startup:
  - `begin()`
  - `arm()`
  - `disarm()`
  - `stop()` or `stopAll()`
- Throttle:
  - `spinRaw(raw)` / `setMotorRaw(index, raw)`
  - `spinPercent(percent)` / `setMotorPercent(index, percent)`
  - `setAllRaw(raw)` / `setAllPercent(percent)`
- Timing:
  - `setTimeoutMs(ms)`
  - `setRefreshMs(ms)`
  - `setHoldArmOnSignalTimeout(bool)`
- Direction at object creation:
  - `EasyEscMotor(..., reversed)`
  - `EasyEsc(..., m1Reversed, m2Reversed, m3Reversed, m4Reversed)`
- Current:
  - `readCurrent()`
  - `currentAmps()`
  - `currentMilliamps()`
  - `currentMilliVolts()`
  - `setCurrentCalibration(zeroMv, mvPerAmp)`
  - `calibrateCurrentZero(samples, sampleDelayMs)`
- Passthrough:
  - `enterPassthrough()`
  - `exitPassthrough()`
  - `passthroughRead(...)`
  - `passthroughWrite(...)`
  - `passthroughErase(...)`

## Interval Units

- `timeoutMs`: milliseconds
- `refreshMs`: milliseconds
- passthrough timing fields: microseconds

## DShot Rules

- Valid throttle command is `0` (stop) or `48..2047` (run range).
- This project default is `DSHOT300`.
- Direction is applied during `begin()` via DShot spin-direction commands.

## Status Codes

`lastStatus()` returns:

- `0 = Ok`
- `1 = InvalidArg`
- `2 = NotInitialized`
- `3 = NotArmed`
- `4 = PassthroughActive`
- `5 = NotInPassthroughMode`
- `6 = DriverError`

## Current Monitor Notes

- Use the ESC CRT/current output pin connected to an ADC-capable ESP32 pin.
- If reading is noisy, run zero calibration with motors stopped:

```cpp
MOTOR1.calibrateCurrentZero(200, 2);
```

## Passthrough Status (Important)

Passthrough support is included but still experimental:

- BLHeli_S bootloader sequence is incomplete
- response CRC/framing validation is incomplete
- read/write/erase behavior is scaffold-level

See [`lib/esc_subsystem/src/esc_passthrough.cpp`](lib/esc_subsystem/src/esc_passthrough.cpp).

## Common Mistakes

- Forgetting to call `update()` in `loop()`
- Sending throttle before `arm()`
- Using non-output GPIO for motor signal
- Using non-ADC pin for current monitor
- Testing with propellers attached

## License and Attribution

See [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).
Portions from Flight32 and DShotRMT are MIT-licensed.
