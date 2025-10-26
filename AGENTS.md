# Repository Guidelines

## Project Structure & Module Organization
- Firmware lives in `WAVE_ROVER_V0.95/` with the main sketch `WAVE_ROVER_V0.95.ino`.
- Feature modules are split by header/source pairs (for example: `IMU.h/.cpp`, `wifi_ctrl.h`, `http_server.h`, `ugv_advance.h`). Keep new modules in this folder and include them from the sketch.
- Web/config assets for LittleFS are under `WAVE_ROVER_V0.95/data/` (for example: `wifiConfig.json`, `devConfig.json`).
- Hardware docs and images (for example: `general_driver_board_schematic.png`) are kept alongside the firmware.

## Build, Upload, and Run
- Arduino IDE: open `WAVE_ROVER_V0.95/WAVE_ROVER_V0.95.ino`, select your ESP32 board, then Verify/Upload.
- Arduino CLI (replace FQBN and port for your board):
  - `arduino-cli core install esp32:esp32`
  - `arduino-cli compile --fqbn esp32:esp32:esp32 WAVE_ROVER_V0.95`
  - `arduino-cli upload -p COM5 --fqbn esp32:esp32:esp32 WAVE_ROVER_V0.95`
- LittleFS data: use the “ESP32 LittleFS Data Upload” tool to flash `WAVE_ROVER_V0.95/data/` to the device.

## Coding Style & Naming Conventions
- Language: Arduino/C++ (ESP32). Prefer C++14-compatible code.
- Indentation: 2 spaces; no tabs. Line length ~100 chars.
- Files: one module per feature; headers `.h`, sources `.cpp`; add `#pragma once` to headers.
- Naming: snake_case for functions/variables, PascalCase for types, UPPER_SNAKE for macros/constants.
- Includes: standard/Arduino first, then external libs, then local headers.

## Testing Guidelines
- No formal unit tests yet. Add smoke checks via Serial logs and sensor reads.
- Verify critical flows: IMU init, WiFi/AP setup, HTTP server endpoints, ESP-NOW modes.
- Use Serial Monitor at `115200` and capture logs with steps to reproduce.

## Commit & Pull Request Guidelines
- Commits: imperative mood, concise scope (for example: `fix(imu): guard null data`); group related changes.
- PRs: clear description, linked issue, screenshots/serial logs when applicable, and notes about hardware, board, and libraries used.
- Keep diffs focused; include brief rationale in code comments only where behavior is non-obvious.

## Security & Configuration Tips
- Do not commit real credentials. Keep `data/wifiConfig.json` and similar files sanitized; share examples only.
- Avoid device-specific constants in code; prefer configuration in `data/*.json` with safe defaults.

### LittleFS (Arduino IDE How‑to)
- Install the "ESP32 LittleFS Data Upload" tool compatible with your Arduino IDE version (see ESP32 core docs).
- Place assets in `WAVE_ROVER_V0.95/data/` (JSON, web files, etc.).
- Connect the ESP32, select the correct Board and Port.
- In Arduino IDE, run `Tools > ESP32 LittleFS Data Upload` and wait for success.
- Open Serial Monitor at `115200` and reboot; look for "Initialize LittleFS" from `initFS()`.
- Repeat the upload whenever files in `data/` change.
