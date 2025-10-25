#ifndef UGV_LED_CTRL_H
#define UGV_LED_CTRL_H

#include <Arduino.h>
#include "ugv_config.h"

/*
  ESP32 Arduino core 3.x LEDC API:
    ledcAttach(pin, freq_hz, resolution_bits);
    ledcWrite(pin, duty);

  We no longer manually create/select channels (IO4_CH, IO5_CH).
  Duty is still 0..(2^resolution_bits - 1).
*/

static inline void led_pin_init() {
    pinMode(IO4_PIN, OUTPUT);
    pinMode(IO5_PIN, OUTPUT);

    /* Set up PWM on each LED pin */
    ledcAttach(IO4_PIN, FREQ, ANALOG_WRITE_BITS);
    ledcAttach(IO5_PIN, FREQ, ANALOG_WRITE_BITS);
}

static inline void led_pwm_ctrl(int io4Input, int io5Input) {
    /* With ANALOG_WRITE_BITS == 8:
       valid duty range = 0..255
    */
    int duty4 = constrain(io4Input, 0, 255);
    int duty5 = constrain(io5Input, 0, 255);

    ledcWrite(IO4_PIN, duty4);
    ledcWrite(IO5_PIN, duty5);
}

#endif /* UGV_LED_CTRL_H */
