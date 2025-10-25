# Waveshare-Rover-General-Driver-Version-
This is an updated vserion of the demo code for the waveshare rover.  The current version from waveshare uses depreciated libraries and was not compatible with the ESP32 core version 3.X

Main Changes from Waveshare Demo Version

- The original code used the ESP-NOW callback style from older ESP32 cores. This has been modified for compatibility with core 3.x / ESP-IDF 5.x.
- The newer INA219_WE library required that we rename enums and rename the power read API. The new version of the linrary library also deprecated BIT_MODE_9, PG_320, BRNG_16. We now use INA219_BIT_MODE_9, INA219_PG_320, INA219_BRNG_16.
- The original code was written for the ESP32 Arduino core 2.x using the old LEDC API. In core 3.x ledcSetup and ledcAttachPin were removed. Channel-based control was replaced with a pin-based API. The code is now compatible with that.
