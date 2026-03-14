/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */
// RTC initialization for stm32f407 board with magic number check
// 14.03.2026

#include "stm32f407xx.h"
#include "rtc.h"

#define RTC_TIMEOUT         1000000U

const uint32_t rtc_magic_number = 0xF55FA00AU;

void RTC_init(void)
{
    uint32_t timeout;

    // Разрешаем доступ к Backup Domain
    if (!(PWR->CR & PWR_CR_DBP)) {
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_DBP;
    }

    // Проверка магического числа
    if (RTC->BKP0R == rtc_magic_number) {
        // RTC уже инициализирован – только включаем, если выключен
        if (!(RCC->BDCR & RCC_BDCR_RTCEN)) {
            RCC->BDCR |= RCC_BDCR_RTCEN;
        }
        // Ждём синхронизации
        timeout = RTC_TIMEOUT;
        while (!(RTC->ISR & RTC_ISR_RSF) && timeout--) {
            __NOP();
        }
        return;
    }

    // ---------- Первый запуск: полная инициализация ----------
    // Пытаемся запустить LSE
    RCC->BDCR |= RCC_BDCR_LSEON;
    timeout = RTC_TIMEOUT * 2;
    while (!(RCC->BDCR & RCC_BDCR_LSERDY) && timeout--) {
        __NOP();
    }

    if (RCC->BDCR & RCC_BDCR_LSERDY) {
        RCC->BDCR |= RCC_BDCR_RTCSEL_0;   // RTCSEL = 01 (LSE)
    } else {
        // LSE не запустился – используем LSI
        RCC->CSR |= RCC_CSR_LSION;
        timeout = RTC_TIMEOUT;
        while (!(RCC->CSR & RCC_CSR_LSIRDY) && timeout--) {
            __NOP();
        }
        RCC->BDCR |= RCC_BDCR_RTCSEL_1;   // RTCSEL = 10 (LSI)
    }
    RCC->BDCR |= RCC_BDCR_RTCEN;           // Включаем RTC

    // Ждём синхронизации
    timeout = RTC_TIMEOUT;
    while (!(RTC->ISR & RTC_ISR_RSF) && timeout--) {
        __NOP();
    }

    // Вход в режим инициализации
    RTC->ISR |= RTC_ISR_INIT;
    timeout = RTC_TIMEOUT;
    while (!(RTC->ISR & RTC_ISR_INITF) && timeout--) {
        __NOP();
    }

    // Настройка предделителей: асинхронный = 127, синхронный = 255
    RTC->PRER = (127UL << 16) | 255UL;

    // 24-часовой формат
    RTC->CR &= ~RTC_CR_FMT;

    // Выход из инициализации
    RTC->ISR &= ~RTC_ISR_INIT;

    // Ждём синхронизации
    timeout = RTC_TIMEOUT;
    while (!(RTC->ISR & RTC_ISR_RSF) && timeout--) {
        __NOP();
    }

    // Записываем магическое число
    RTC->BKP0R = rtc_magic_number;
}
