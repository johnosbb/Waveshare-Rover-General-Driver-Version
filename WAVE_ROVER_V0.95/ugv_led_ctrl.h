#ifndef UGV_LED_CTRL_H
#define UGV_LED_CTRL_H

#include <Arduino.h>
#include "ugv_config.h"
#include "config.h"

/*
  ESP32 Arduino core 3.x LEDC API:
    ledcAttach(pin, freq_hz, resolution_bits);
    ledcWrite(pin, duty);

  We no longer manually create/select channels (IO4_CH, IO5_CH).
  Duty is still 0..(2^resolution_bits - 1).
*/

static bool led4_ready = false;
static bool led5_ready = false;

static inline void led_pin_init() {
#if !ENABLE_LEDS
    return;
#endif
    bool conflict4 = false;
    bool conflict5 = false;
#if ENABLE_USER_UART
    if (IO4_PIN == USER_UART_RX_PIN || IO4_PIN == USER_UART_TX_PIN) conflict4 = true;
    if (IO5_PIN == USER_UART_RX_PIN || IO5_PIN == USER_UART_TX_PIN) conflict5 = true;
#endif

    if (!conflict4) {
        pinMode(IO4_PIN, OUTPUT);
        ledcAttach(IO4_PIN, FREQ, ANALOG_WRITE_BITS);
        led4_ready = true;
    }
    if (!conflict5) {
        pinMode(IO5_PIN, OUTPUT);
        ledcAttach(IO5_PIN, FREQ, ANALOG_WRITE_BITS);
        led5_ready = true;
    }
}

static inline void led_pwm_ctrl(int io4Input, int io5Input) {
#if !ENABLE_LEDS
    (void)io4Input; (void)io5Input; return;
#endif
    /* With ANALOG_WRITE_BITS == 8:
       valid duty range = 0..255
    */
    int duty4 = constrain(io4Input, 0, 255);
    int duty5 = constrain(io5Input, 0, 255);

    if (led4_ready) { ledcWrite(IO4_PIN, duty4); }
    if (led5_ready) { ledcWrite(IO5_PIN, duty5); }
}

#endif /* UGV_LED_CTRL_H */
