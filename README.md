# Waveshare-Rover-General-Driver-Version-
This is an updated vserion of the demo code for the waveshare rover.  The current version from waveshare uses depreciated libraries and was not compatible with the ESP32 core version 3.X

Main Changes from Waveshare Demo Version

- The original code used the ESP-NOW callback style from older ESP32 cores. This has been modified for compatibility with core 3.x / ESP-IDF 5.x.
- The newer INA219_WE library required that we rename enums and rename the power read API. The new version of the linrary library also deprecated BIT_MODE_9, PG_320, BRNG_16. We now use INA219_BIT_MODE_9, INA219_PG_320, INA219_BRNG_16.
- The original code was written for the ESP32 Arduino core 2.x using the old LEDC API. In core 3.x ledcSetup and ledcAttachPin were removed. Channel-based control was replaced with a pin-based API. The code is now compatible with that.
- Support for ESP-NOW can now be enabled or disabled by setting ENABLE_ESP_NOW (default 0). Set to 1 to include ESP-NOW.
- The Robotic Arm (RoArm-M2) can be compiled in or out via ENABLE_ROBOTIC_ARM (default 1). Set to 0 to exclude all arm logic for platforms without the arm.


## General Driver Board


<img width="818" height="422" alt="image" src="https://github.com/user-attachments/assets/a7adb55c-7ae2-4432-b59f-fcfe7c97e77f" />

| No. | Onboard Resources                      | Description                                                                 |
|-----|----------------------------------------|------------------------------------------------------------------------------|
| 1   | ESP32-WROOM-32 main controller         | Can be developed with the Arduino IDE                                       |
| 2   | IPEX1 WiFi connector                   | For connecting the WIFI antenna to increase the wireless communication distance |
| 3   | LIDAR interface                        | Integrated radar adapter board function                                     |
| 4   | IIC peripheral expansion interface     | Can be used to connect OLED screens or other IIC sensors                    |
| 5   | Reset button                           | Press and release to reboot the ESP32                                       |
| 6   | Download button                        | The ESP32 will enter the download mode when the power is turned on by pressing |
| 7   | DC-DC 5V voltage regulator circuit     | Power supply for host computers such as Raspberry Pi or Jetson Nano         |
| 8   | Type-C connector (LIDAR)               | LIDAR data interface                                                        |
| 9   | Type-C connector (USB)                 | ESP32 UART communication interface, can upload programs for ESP32           |
| 10  | XH2.54 power port                      | Input DC 7~13V, this interface directly powers the serial bus servo and motor |
| 11  | INA219                                 | Voltage/current monitoring chip                                             |
| 12  | Power ON/OFF                           | Switches to control external power supply                                   |
| 13  | ST3215 serial bus servo interface      | For connecting to ST3215 serial bus servo                                   |
| 14  | Motor interface PH2.0 6P               | Group B interface for motor with encoder                                    |
| 15  | Motor interface PH2.0 6P               | Group A interface for motor with encoder                                    |
| 16  | Motor interface PH2.0 2P               | Group A interface for motor without encoder                                 |
| 17  | Motor interface PH2.0 2P               | Group B interface for motor without encoder                                 |
| 18  | AK09918C                               | 3-axis electronic compass                                                   |
| 19  | QMI8658                                | 6-axis motion sensor                                                        |
| 20  | TB6612FNG                              | Motor control chip                                                          |
| 21  | Serial bus servo control circuit       | Can be used to expand multiple ST3215 serial bus servos and obtain servo feedback |
| 22  | SD card slot                           | Can be used to store logs or WIFI configurations                            |
| 23  | 40PIN extended header                  | Easy access to Raspberry Pi or Horizon Sunrise X3 Pi                        |
| 24  | 40PIN extended header                  | Easy to use the pins of the host computer installed on the driver board     |
| 25  | CP2102                                 | UART to USB for radar data transfer                                         |
| 26  | CP2102                                 | UART to USB for ESP32 UART communication                                    |
| 27  | Automatic download circuit             | Upload demos for the ESP32 without pressing the EN and BOOT buttons         |


## Feature Inventory (Functional Blocks)

This repository organizes firmware under `WAVE_ROVER_V0.95/` with the sketch `WAVE_ROVER_V0.95.ino`. Below is a concise inventory of the main functional blocks, the files they live in, and whether they are optional/hardware‑specific.

### Core Sketch and Configuration
- `WAVE_ROVER_V0.95/WAVE_ROVER_V0.95.ino`: Integrates all modules; sets up OLED, IMU, WiFi, HTTP server, motion, PID, and optional modules.
- `WAVE_ROVER_V0.95/ugv_config.h`: Central hardware/pin map, kinematic constants for the arm, motor encoder parameters, PWM settings, and global runtime variables.
- `WAVE_ROVER_V0.95/config.h`: Build‑time feature toggles (currently `ENABLE_ESP_NOW`).

### Drive Base (Motors/Encoders/PID)
- `WAVE_ROVER_V0.95/movtion_module.h`: Motor driver (LEDC PWM), H‑bridge direction control, encoder readout (`ESP32Encoder`), closed‑loop velocity via `PID_v2`, ROS‑style velocity input (`CMD_ROS_CTRL`), and speed scaling.
- `WAVE_ROVER_V0.95/ugv_led_ctrl.h`: PWM control for auxiliary LEDs on `IO4/IO5`.

### Power & Display
- `WAVE_ROVER_V0.95/battery_ctrl.h`: INA219 monitor for shunt/bus voltage, current, and power.
- `WAVE_ROVER_V0.95/oled_ctrl.h`: SSD1306 0.91" OLED, four status lines, default view shows version, Wi‑Fi mode/IP, and supply voltage.

### IMU (QMI8658 + AK09918)
- `WAVE_ROVER_V0.95/IMU.h/.cpp`, `QMI8658*.{h,cpp}`, `AK09918*.{h,cpp}`: Sensor drivers and an AHRS implementation producing roll/pitch/yaw and raw accel/gyro/mag.
- `WAVE_ROVER_V0.95/IMU_ctrl.h`: Wrapper to initialize, update globals, and serialize IMU data to JSON feedback.

### Networking & Web UI
- `WAVE_ROVER_V0.95/wifi_ctrl.h`: Wi‑Fi AP/STA/AP+STA modes, config from `LittleFS:/wifiConfig.json`, status reporting; updates OLED lines accordingly.
- `WAVE_ROVER_V0.95/http_server.h`: Lightweight HTTP server exposing:
  - `/` serving `WAVE_ROVER_V0.95/web_page.h` (inlined UI)
  - `/js` accepting JSON commands and returning JSON feedback
- `WAVE_ROVER_V0.95/data/`: LittleFS assets (`wifiConfig.json`, `devConfig.json`).

### ESP‑NOW (Optional)
- `WAVE_ROVER_V0.95/esp_now_ctrl.h`: Optional peer‑to‑peer control/telemetry. Modes: follower, group leader, single‑device leader. Enable at build by setting `ENABLE_ESP_NOW=1` (see `config.h`).

### Robotic Arm: RoArm‑M2 (Optional Hardware)
- `WAVE_ROVER_V0.95/RoArm-M2_module.h`: SCServo bus control (Serial1 @ 1Mbps), joint state feedback, PID/torque control, homing to init pose, per‑joint and multi‑joint motion, basic kinematics helpers.
- `WAVE_ROVER_V0.95/ugv_advance.h`: Mission scripting (create/append/insert/replace/delete steps), playback with abort on serial input, EoAT configuration, and helpers for JSON‑driven sequences.
- Arm geometry and limits live in `ugv_config.h` (link lengths, angle limits, IDs, etc.).

### Gimbal (Pan/Tilt, Optional Hardware)
- `WAVE_ROVER_V0.95/gimbal_module.h`: Two‑axis gimbal control (pan/tilt IDs in `ugv_config.h`), simple motion and IMU‑assisted steady mode.

### Filesystem & Missions
- `WAVE_ROVER_V0.95/files_ctrl.h`: LittleFS utilities to create/read/append/insert/replace/delete lines; used for mission storage (e.g., `boot.mission`).

### Command Protocol (UART/Web)
- `WAVE_ROVER_V0.95/json_cmd.h`: Command and feedback IDs with inline JSON examples.
- `WAVE_ROVER_V0.95/uart_ctrl.h`: Central JSON dispatcher for all commands from Serial and the `/js` endpoint (drive base, PID, OLED, IMU, Wi‑Fi, ESP‑NOW, missions, arm, gimbal, etc.).

### Hardware Docs
- `WAVE_ROVER_V0.95/general_driver_board_schematic.png`: Board reference schematic.

### Not Present / Notes
- LiDAR: Any LiDAR would require an additional module (likely via UART to a PI or Jettson) and corresponding commands.
- Feature gating: Aside from `ENABLE_ESP_NOW`, modules are always compiled in. If binary size matters for models without the arm/gimbal/OLED/etc., a future step is to add per‑module toggles in `config.h` and wrap includes/logic with `#if` guards.

## Build Flags

Two compile-time toggles live in WAVE_ROVER_V0.95/config.h:
- ENABLE_ESP_NOW (default 0): set to 1 to include ESP-NOW support.
- ENABLE_ROBOTIC_ARM (default 0): set to 0 to exclude Robotic Arm (RoArm-M2) code when no arm is installed.

Behavior when the arm is disabled:
- Arm initialization and control code are not compiled.
- If moduleType is set to 1 (arm) at boot, firmware forces moduleType=0 and logs a notice.

How to set via Arduino CLI:
- Arduino-cli compile --fqbn esp32:esp32:esp32 WAVE_ROVER_V0.95 --build-properties build.extra_flags="-DENABLE_ROBOTIC_ARM=0"

Arduino IDE:
- Edit WAVE_ROVER_V0.95/config.h and set the #define values as needed.
