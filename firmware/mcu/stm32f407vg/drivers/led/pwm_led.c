// pwm_led.c
/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#include "pwm_led.h"
#include "stm32f407xx.h"
#include "micros.h"

#define PWM_PERIOD           256

// Private variables
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

// Set PWM compare value (0..255) with inversion for active-low output
static void set_compare(uint8_t bright) {
    TIM2->CCR2 = bright;
}

// Initialize TIM2 channel 2 on PA1 for PWM (active low, push-pull)
static void pwm_init_hw(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configure PA1 as alternate function AF1 (TIM2)
    GPIOA->MODER &= ~GPIO_MODER_MODER1;
    GPIOA->MODER |= GPIO_MODER_MODER1_1;      // 10 = alternate function
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL1;       // clear AF bits for PA1
    GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFSEL1_Pos); // AF1 for TIM2

    GPIOA->OTYPER &= ~GPIO_OTYPER_OT1;        // push-pull
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED1;   // very high speed
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;       // no pull

    // Reset TIM2
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;

    // Configure TIM2: 1 MHz timer clock (168 MHz / 168)
    TIM2->PSC = 168 - 1;
    TIM2->ARR = PWM_PERIOD - 1;
    TIM2->CNT = 0;

    // PWM mode 1, preload enable, polarity low (active low)
    TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
    TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; // 110 = PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE;
    TIM2->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2P;        // enable, active low

    TIM2->EGR |= TIM_EGR_UG;                    // update registers
    TIM2->CR1 |= TIM_CR1_CEN;                   // start timer
}

// Public: init hardware and set default state
static void pwm_init(void) {
    pwm_init_hw();
    set_compare(0);
    led.mode = LED_MODE_OFF;
    led.last_update = micros();
    led.phase = 0;
}

// Public: set brightness immediately (0..255)
static void pwm_set_brightness(uint8_t brightness) {
    led.current_bright = brightness;
    set_compare(brightness);
}

// Public: set operation mode
static void pwm_set_mode(led_mode_t mode, uint16_t period_ms, uint8_t min_bright, uint8_t max_bright) {
    led.mode = mode;
    led.period_us = (uint32_t)period_ms * 1000;
    led.min_bright = min_bright;
    led.max_bright = max_bright;
    led.last_update = micros();
    led.phase = 0;          // сбрасываем фазу при смене режима

    // Для режимов без периодичности устанавливаем сразу
    switch (mode) {
        case LED_MODE_OFF:
            pwm_set_brightness(0);
            break;
        case LED_MODE_ON:
            pwm_set_brightness(max_bright);
            break;
        default:
            // остальные режимы будут обновляться в proc
            break;
    }
}

// Main processing routine – call frequently
static void pwm_proc(void) {
    uint32_t now = micros();

    // Обработка переполнения micros: если now меньше предыдущего значения,
    // значит счётчик переполнился – просто обновляем last_update и выходим
    if (now < led.last_update) {
        led.last_update = now;
        return;
    }

    uint32_t dt = now - led.last_update;
    if (dt < 1000) return;                 // обновление не чаще 1 мс

    led.last_update = now;

    if (led.period_us == 0) return;        // для режимов OFF/ON ничего не делаем

    // Обновляем фазу: добавляем dt и приводим к диапазону [0, period_us-1]
    led.phase = (led.phase + dt) % led.period_us;

    uint32_t t = led.phase;   // текущая позиция внутри периода
    uint8_t new_bright = led.current_bright;

    switch (led.mode) {
        case LED_MODE_OFF:
        case LED_MODE_ON:
            // ничего не делаем
            break;

        case LED_MODE_BLINK:
            // прямоугольные импульсы: половина периода вкл, половина выкл
            new_bright = (t < led.period_us / 2) ? led.max_bright : led.min_bright;
            break;

        case LED_MODE_BREATHE: {
            uint32_t half = led.period_us / 2;
            if (t < half) {
                uint32_t progress = (t * 255) / half;   // 0..255
                new_bright = (uint8_t)(led.min_bright + (led.max_bright - led.min_bright) * progress / 255);
            } else {
                uint32_t progress = ((t - half) * 255) / half;
                new_bright = (uint8_t)(led.max_bright - (led.max_bright - led.min_bright) * progress / 255);
            }
            break;
        }

        case LED_MODE_FADE_IN:
            // линейное нарастание от min до max в течение периода
            new_bright = (uint8_t)(led.min_bright + (led.max_bright - led.min_bright) * t / led.period_us);
            break;

        case LED_MODE_FADE_OUT:
            // линейное затухание от max до min в течение периода
            new_bright = (uint8_t)(led.max_bright - (led.max_bright - led.min_bright) * t / led.period_us);
            break;

        default:
            break;
    }

    if (new_bright != led.current_bright) {
        pwm_set_brightness(new_bright);
    }
}

// Public interface
const pwm_led_t pwm_led = {
    .init = pwm_init,
    .set_mode = pwm_set_mode,
    .set_brightness = pwm_set_brightness,
    .proc = pwm_proc
};