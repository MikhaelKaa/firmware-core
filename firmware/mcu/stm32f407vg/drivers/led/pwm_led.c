// pwm_led.c
/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

#include "pwm_led.h"
#include "stm32f407xx.h"
#include "micros.h"          // provides micros()

// PWM resolution (0..255)
#define PWM_PERIOD           256

// Private variables
static volatile struct {
    led_mode_t mode;
    uint32_t   period_us;           // full cycle in microseconds
    uint8_t    min_bright;
    uint8_t    max_bright;
    uint8_t    current_bright;
    uint32_t   last_update;          // last micros() timestamp
    int8_t     breathe_dir;          // +1 or -1 for BREATHE mode
} led = {
    .mode = LED_MODE_OFF,
    .period_us = 1000000,            // 1 sec default
    .min_bright = 0,
    .max_bright = 255,
    .current_bright = 0,
    .last_update = 0,
    .breathe_dir = 1
};

// Set PWM compare value (0..255) with inversion for active-low output
static void set_compare(uint8_t bright) {
    // TIM2_CH2 compare register
    TIM2->CCR2 = bright;   // with PWM1, polarity low: bright=0 -> off, bright=255 -> full on
}

// Initialize TIM2 channel 2 on PA1 for PWM (active low, push-pull)
static void pwm_init_hw(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configure PA1 as alternate function AF1 (TIM2)
    GPIOA->MODER &= ~GPIO_MODER_MODER1;
    GPIOA->MODER |= GPIO_MODER_MODER1_1;      // 10 = alternate function
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL1) | (1 << GPIO_AFRL_AFSEL1_Pos);

    // Push-pull, high speed
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT1;
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED1;   // very high speed

    // No pull-up/pull-down
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;

    // Reset TIM2
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;

    // Configure TIM2
    // Prescaler: 168 MHz / 168 = 1 MHz (1 us per tick)
    TIM2->PSC = 168 - 1;                      // assuming 168 MHz system clock
    // Auto-reload: PWM_PERIOD - 1
    TIM2->ARR = PWM_PERIOD - 1;
    // Clear counter
    TIM2->CNT = 0;

    // PWM mode 1, preload enable, polarity low (active low)
    TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
    TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; // 110 = PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE;            // preload enable
    TIM2->CCER |= TIM_CCER_CC2E;                // output enable
    TIM2->CCER |= TIM_CCER_CC2P;                // polarity low (active low)

    // Generate update event to load registers
    TIM2->EGR |= TIM_EGR_UG;

    // Enable counter
    TIM2->CR1 |= TIM_CR1_CEN;
}

// Public: init hardware and set default state
static void pwm_init(void) {
    pwm_init_hw();
    set_compare(0);
    led.mode = LED_MODE_OFF;
    led.last_update = micros();
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

    // For BREATHE, initial direction
    if (mode == LED_MODE_BREATHE) {
        led.breathe_dir = 1;
    }

    // For immediate modes, set initial brightness
    switch (mode) {
        case LED_MODE_OFF:
            pwm_set_brightness(0);
            break;
        case LED_MODE_ON:
            pwm_set_brightness(max_bright);
            break;
        default:
            // other modes will be handled in proc
            break;
    }
}

// Main processing routine – call this frequently (e.g., in main loop)
static void pwm_proc(void) {
    uint32_t now = micros();
    uint32_t dt = now - led.last_update;
    uint8_t new_bright = led.current_bright;
    uint32_t t;

    if (dt < 1000) return;

    led.last_update = now;

    switch (led.mode) {
        case LED_MODE_OFF:
        case LED_MODE_ON:
            break;

        case LED_MODE_BLINK:
            t = now % led.period_us;
            new_bright = (t < led.period_us / 2) ? led.max_bright : led.min_bright;
            break;

        case LED_MODE_BREATHE: {
            uint32_t half = led.period_us / 2;
            t = now % led.period_us;
            if (t < half) {
                uint32_t progress = (t * 255) / half;
                new_bright = (uint8_t)(led.min_bright + (led.max_bright - led.min_bright) * progress / 255);
            } else {
                uint32_t progress = ((t - half) * 255) / half;
                new_bright = (uint8_t)(led.max_bright - (led.max_bright - led.min_bright) * progress / 255);
            }
            break;
        }

        case LED_MODE_FADE_IN:
            t = now % led.period_us;
            if (t < led.period_us) {
                uint32_t progress = (t * 255) / led.period_us;
                new_bright = (uint8_t)(led.min_bright + (led.max_bright - led.min_bright) * progress / 255);
            } else {
                new_bright = led.max_bright;
            }
            break;

        case LED_MODE_FADE_OUT:
            t = now % led.period_us;
            if (t < led.period_us) {
                uint32_t progress = (t * 255) / led.period_us;
                new_bright = (uint8_t)(led.max_bright - (led.max_bright - led.min_bright) * progress / 255);
            } else {
                new_bright = led.min_bright;
            }
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