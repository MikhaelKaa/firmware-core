// pwm_led.h
#ifndef PWM_LED_H
#define PWM_LED_H

#include <stdint.h>

typedef enum {
    LED_MODE_OFF,
    LED_MODE_ON,
    LED_MODE_BLINK,
    LED_MODE_BREATHE,
    LED_MODE_FADE_IN,
    LED_MODE_FADE_OUT
} led_mode_t;

typedef struct {
    void (*init)(void);
    void (*set_mode)(led_mode_t mode, uint16_t period_ms, uint8_t min_bright, uint8_t max_bright);
    void (*set_brightness)(uint8_t brightness);   // 0..255
    void (*proc)(void);                            // call in main loop
} pwm_led_t;

extern const pwm_led_t pwm_led;

#endif