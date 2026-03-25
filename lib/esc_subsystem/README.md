# esc_subsystem

Core ESC library used by this project.

Main public header:

```cpp
#include <easy_esc.h>
```

## Main Classes

- `esc::EasyEscMotor`: single-motor object, simplest API
- `esc::EasyEsc`: multi-motor object (1 to 4 motors)

## Object Creation

Single motor:

```cpp
esc::EasyEscMotor MOTOR1(GPIO_NUM_41, GPIO_NUM_3, DSHOT300, 20000, 10, 0.0f, 33.0f, true, false);
```

Four motors:

```cpp
esc::EasyEsc ESC4(
  4,
  GPIO_NUM_41, GPIO_NUM_42, GPIO_NUM_40, GPIO_NUM_39,
  GPIO_NUM_3,
  DSHOT300,
  20000,
  10,
  0.0f,
  33.0f,
  false, true, false, false,
  false
);
```

Set the final constructor argument to `true` to enable bidirectional DShot.

## Required Runtime Flow

1. `begin()`
2. `arm()`
3. call `update()` continuously in `loop()`
4. send throttle (`spinRaw`, `setMotorRaw`, `setAllRaw`)
5. `disarm()` or `stop()` when done

## Timing Configuration

- `setTimeoutMs(ms)`: signal timeout (milliseconds)
- `setRefreshMs(ms)`: output keepalive resend interval (milliseconds)
- `setHoldArmOnSignalTimeout(true/false)`: choose timeout behavior

If hold-on-timeout is enabled, timeout forces zero throttle but stays armed.

## DShot Defaults

- default mode: `DSHOT300`
- valid throttle range: `0` or `48..2047`
- bidirectional DShot option exists and defaults to `false`
- direction is configurable at object creation and applied during `begin()`

## Current Monitor

Available helpers:

- `readCurrent()`
- `currentAmps()`
- `currentMilliamps()`
- `currentMilliVolts()`
- `setCurrentCalibration(zeroMv, mvPerAmp)`
- `calibrateCurrentZero(samples, sampleDelayMs)`

## Passthrough

Passthrough methods exist, but this part is experimental:

- incomplete BLHeli_S bootloader flow
- incomplete CRC/framing validation
- scaffold-level read/write/erase behavior
