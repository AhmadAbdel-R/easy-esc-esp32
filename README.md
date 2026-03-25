# easy-esc-esp32

Minimal ESP32 ESC library with:

- DShot motor output (default: `DSHOT300`)
- arm/disarm safety
- optional current monitor input
- experimental BLHeli_S / Bluejay passthrough scaffold

## Target setup

- ESC hardware: `Flycolor Raptor BLS-04 4-in-1`
- ESC firmware: set in [`src/main.cpp`](src/main.cpp) (`kEscFirmwareName`)
- Compatible stacks: `BLHeli_S / Bluejay`

## Quick start

1. Remove propellers.
2. Open project in PlatformIO.
3. Set your pins in [`src/main.cpp`](src/main.cpp).
4. Build and upload.
5. Open serial monitor at `115200`.

## Default example

[`src/main.cpp`](src/main.cpp) demonstrates:

- `esc::EasyEscMotor` with servo-style API
- potentiometer to raw DShot mapping (`0` or `48..2047`)
- current monitor readout
- startup safety gate and arming flow

## API (EasyEscMotor)

- `begin()`
- `update()`
- `arm()`
- `disarm()`
- `stop()`
- `spinRaw(raw)`
- `spinPercent(percent)`
- `setTimeoutMs(ms)`
- `setRefreshMs(ms)`
- `setHoldArmOnSignalTimeout(bool)`
- `readCurrent()`
- `currentAmps()`
- `currentMilliamps()`
- `currentMilliVolts()`
- `setCurrentCalibration(zero_mv, mv_per_a)`
- `calibrateCurrentZero(samples, delayMs)`
- `enterPassthrough()`
- `exitPassthrough()`

## Interval units

- `signalTimeoutMs`: milliseconds
- `outputRefreshMs`: milliseconds
- passthrough timing fields (`resetUs`, `halfBitDelayUs`, etc.): microseconds

## Passthrough status

Passthrough is experimental in this extraction:

- full BLHeli_S bootloader sequence is not complete
- response CRC/framing validation is incomplete
- read/write/erase paths are scaffold-level

See [`lib/esc_subsystem/src/esc_passthrough.cpp`](lib/esc_subsystem/src/esc_passthrough.cpp).

## License and attribution

See [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).
Portions from Flight32 and DShotRMT are MIT-licensed.
