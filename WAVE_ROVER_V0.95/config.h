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
