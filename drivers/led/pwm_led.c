// pwm_led_drv.c
/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#include "pwm_led.h"
#include "stm32f407xx.h"
#include "micros.h"
#include <errno.h>

#define PWM_PERIOD           256

// Состояние драйвера (одно на весь модуль, так как управляем одним светодиодом)
static volatile struct {
    led_mode_t mode;
    uint32_t   period_us;           // полный цикл в микросекундах
    uint8_t    min_bright;
    uint8_t    max_bright;
    uint8_t    current_bright;
    uint32_t   last_update;         // последний вызов micros()
    uint32_t   phase;                // текущая фаза (0 .. period_us-1)
} led = {
    .mode = LED_MODE_OFF,
    .period_us = 1000000,
    .min_bright = 0,
    .max_bright = 255,
    .current_bright = 0,
    .last_update = 0,
    .phase = 0
};

// Установка значения сравнения ШИМ (аппаратная)
static void set_compare(uint8_t bright) {
    TIM2->CCR2 = bright;
}

// Инициализация аппаратуры (GPIO, таймер)
static void pwm_init_hw(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // PA1 as AF1 (TIM2)
    GPIOA->MODER &= ~GPIO_MODER_MODER1;
    GPIOA->MODER |= GPIO_MODER_MODER1_1;      // alternate function
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL1;
    GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFSEL1_Pos); // AF1

    GPIOA->OTYPER &= ~GPIO_OTYPER_OT1;        // push-pull
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED1;   // high speed
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;       // no pull

    // Reset TIM2
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;

    // TIM2 config: 1 MHz timer clock (168 MHz / 168)
    TIM2->PSC = 168 - 1;
    TIM2->ARR = PWM_PERIOD - 1;
    TIM2->CNT = 0;

    // PWM mode 1, preload enable, active low
    TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
    TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; // 110 = PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE;
    TIM2->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2P;        // enable, active low

    TIM2->EGR |= TIM_EGR_UG;                    // update registers
    TIM2->CR1 |= TIM_CR1_CEN;                   // start timer
}

// Установка яркости (мгновенная)
static void pwm_set_brightness(uint8_t brightness) {
    led.current_bright = brightness;
    set_compare(brightness);
}

// Установка режима работы
static void pwm_set_mode(led_mode_t mode, uint16_t period_ms,
                         uint8_t min_bright, uint8_t max_bright) {
    led.mode = mode;
    led.period_us = (uint32_t)period_ms * 1000;
    led.min_bright = min_bright;
    led.max_bright = max_bright;
    led.last_update = micros();
    led.phase = 0;

    // Для статических режимов сразу устанавливаем яркость
    switch (mode) {
        case LED_MODE_OFF:
            pwm_set_brightness(0);
            break;
        case LED_MODE_ON:
            pwm_set_brightness(max_bright);
            break;
        default:
            break;
    }
}

// Периодическая обработка (вызывается из главного цикла)
static void pwm_proc(void) {
    uint32_t now = micros();

    // Обработка переполнения micros()
    if (now < led.last_update) {
        led.last_update = now;
        return;
    }

    uint32_t dt = now - led.last_update;
    if (dt < 1000) return;                 // обновление не чаще 1 мс
    led.last_update = now;

    if (led.period_us == 0) return;        // для OFF/ON ничего не делаем

    led.phase = (led.phase + dt) % led.period_us;
    uint32_t t = led.phase;
    uint8_t new_bright = led.current_bright;

    switch (led.mode) {
        case LED_MODE_OFF:
        case LED_MODE_ON:
            break;

        case LED_MODE_BLINK:
            new_bright = (t < led.period_us / 2) ? led.max_bright : led.min_bright;
            break;

        case LED_MODE_BREATHE: {
            uint32_t half = led.period_us / 2;
            if (t < half) {
                uint32_t progress = (t * 255) / half;
                new_bright = (uint8_t)(led.min_bright +
                              (led.max_bright - led.min_bright) * progress / 255);
            } else {
                uint32_t progress = ((t - half) * 255) / half;
                new_bright = (uint8_t)(led.max_bright -
                              (led.max_bright - led.min_bright) * progress / 255);
            }
            break;
        }

        case LED_MODE_FADE_IN:
            new_bright = (uint8_t)(led.min_bright +
                          (led.max_bright - led.min_bright) * t / led.period_us);
            break;

        case LED_MODE_FADE_OUT:
            new_bright = (uint8_t)(led.max_bright -
                          (led.max_bright - led.min_bright) * t / led.period_us);
            break;

        default:
            break;
    }

    if (new_bright != led.current_bright) {
        pwm_set_brightness(new_bright);
    }
}

// Реализация интерфейса drv_face_t

static int pwm_led_read(void *buf, size_t len) {
    if (len < 1) return -EINVAL;
    *(uint8_t*)buf = led.current_bright;
    return 1;
}

static int pwm_led_write(const void *buf, size_t len) {
    if (len < 1) return -EINVAL;
    pwm_set_brightness(*(const uint8_t*)buf);
    return 1;
}

static int pwm_led_ioctl(int cmd, void *arg) {
    switch (cmd) {
        case INTERFACE_INIT:
            pwm_init_hw();
            pwm_set_brightness(0);
            led.mode = LED_MODE_OFF;
            led.last_update = micros();
            led.phase = 0;
            return 0;

        case INTERFACE_DEINIT:
            // Выключение таймера и сброс GPIO (опционально)
            TIM2->CR1 &= ~TIM_CR1_CEN;
            GPIOA->MODER &= ~GPIO_MODER_MODER1; // вернуть в аналоговый/вход
            return 0;

        case INTERFACE_GET_PROC:
            *(void(**)(void))arg = pwm_proc;
            return 0;

        case PWM_LED_CMD_SET_BRIGHTNESS: {
            const pwm_led_set_brightness_t *p = arg;
            pwm_set_brightness(p->brightness);
            return 0;
        }

        case PWM_LED_CMD_SET_MODE: {
            const pwm_led_set_mode_t *p = arg;
            pwm_set_mode(p->mode, p->period_ms, p->min_bright, p->max_bright);
            return 0;
        }

        default:
            return -ENOTSUP;
    }
}

// Экспортируемый экземпляр драйвера
const drv_face_t pwm_led_dev = {
    .read = pwm_led_read,
    .write = pwm_led_write,
    .ioctl = pwm_led_ioctl
};

const drv_face_t* dev_pwm_led_get(void)
{
    return (const drv_face_t*) &pwm_led_dev;
}