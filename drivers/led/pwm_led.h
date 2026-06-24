// pwm_led_drv.h
#ifndef PWM_LED_DRV_H
#define PWM_LED_DRV_H

#include <stdint.h>
#include "drv_face.h"

// Режимы работы светодиода
typedef enum {
    LED_MODE_OFF,
    LED_MODE_ON,
    LED_MODE_BLINK,
    LED_MODE_BREATHE,
    LED_MODE_FADE_IN,
    LED_MODE_FADE_OUT
} led_mode_t;

// Команды ioctl для драйвера светодиода
#define PWM_LED_CMD_SET_BRIGHTNESS  (INTERFACE_CMD_DEVICE | 0x01U)
#define PWM_LED_CMD_SET_MODE        (INTERFACE_CMD_DEVICE | 0x02U)

// Структуры параметров для команд
typedef struct {
    uint8_t brightness;   // Яркость светодиода (0..255)
} pwm_led_set_brightness_t;

typedef struct {
    led_mode_t mode;
    uint16_t period_ms;   // Период в миллисекундах для режимов BLINK, BREATHE, FADE_IN, FADE_OUT
    uint8_t min_bright;   // Минимальная яркость (0..255)
    uint8_t max_bright;   // Максимальная яркость (0..255)
} pwm_led_set_mode_t;

// Экземпляр драйвера PWM LED
extern const drv_face_t pwm_led_dev;
const drv_face_t* dev_pwm_led_get(void);

// Удобные inline-обёртки для вызова ioctl

// Установка яркости светодиода
static inline int pwm_led_set_brightness(drv_face_t *dev, uint8_t brightness) {
    return dev->ioctl(PWM_LED_CMD_SET_BRIGHTNESS,
                      &(pwm_led_set_brightness_t){.brightness = brightness});
}

// Установка режима работы светодиода
static inline int pwm_led_set_mode(drv_face_t *dev,
                                   led_mode_t mode,
                                   uint16_t period_ms,
                                   uint8_t min_bright,
                                   uint8_t max_bright) {
    return dev->ioctl(PWM_LED_CMD_SET_MODE,
                      &(pwm_led_set_mode_t){
                          .mode = mode,
                          .period_ms = period_ms,
                          .min_bright = min_bright,
                          .max_bright = max_bright
                      });
}

#endif // PWM_LED_DRV_H
