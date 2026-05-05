# easy-esc-esp32

ESP32 ESC control library built around DShot + RMT.

This project is meant to be easy to drop into student/capstone projects while still exposing enough control for real testing.

## Core Features

- single-motor wrapper (`EasyEscMotor`)
- multi-motor wrapper (`EasyEsc`)
- arm/disarm safety flow
- timeout and refresh control
- optional current monitor helpers
- passthrough scaffold for ESC tooling (still experimental)

## Repo Layout

- `lib/esc_subsystem/src/` -> core library implementation
- `src/main.cpp` -> example sketch
- `tools/esc_serial_ui.py` -> serial status UI helper

## Current Reality

- DShot output path is usable and recently hardened for RMT recovery.
- Passthrough support is not fully complete yet.

## Build

- Open in PlatformIO and build the configured environment.
