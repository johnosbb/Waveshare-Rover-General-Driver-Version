#pragma once

// Feature toggles for optional modules.
// Set to 1 to include ESP-NOW support; 0 disables it by default.
// You can also define ENABLE_ESP_NOW=1 via compiler flags.
#ifndef ENABLE_ESP_NOW
#define ENABLE_ESP_NOW 0
#endif

// Set to 1 to include Robotic Arm (RoArm-M2) support; 0 excludes it at compile time.
// You can also define ENABLE_ROBOTIC_ARM=0/1 via compiler flags.
#ifndef ENABLE_ROBOTIC_ARM
#define ENABLE_ROBOTIC_ARM 0
#endif

// Enable wheel encoders and PID speed control paths.
// Set to 0 to free GPIO27/GPIO16 and skip encoder reads and PID compute calls.
#ifndef ENABLE_ENCODERS
#define ENABLE_ENCODERS 1
#endif

// Enable IO4/IO5 LED PWM helpers. If USER UART is enabled and shares IO4/IO5,
// the LED helpers will automatically skip conflicting pins at runtime.
#ifndef ENABLE_LEDS
#define ENABLE_LEDS 1
#endif

// Optional user UART (Serial2) for external peripherals/hosts.
// When enabled, Serial2 is initialized with the pins and baud below.
#ifndef ENABLE_USER_UART
#define ENABLE_USER_UART 0
#endif

#ifndef USER_UART_RX_PIN
// Defaults target the center Dupont header: IO4 (RX)
#define USER_UART_RX_PIN 4
#endif

#ifndef USER_UART_TX_PIN
// Defaults target the center Dupont header: IO5 (TX)
#define USER_UART_TX_PIN 5
#endif

#ifndef USER_UART_BAUD
#define USER_UART_BAUD 115200
#endif
