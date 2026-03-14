/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */
// RTC initialization for stm32f407 board
// 14.03.2026

#include "stm32f407xx.h"

#define RTC_TIMEOUT     1000000U

void RTC_init(void)
{
    uint32_t timeout;

    // Убедимся, что доступ к Backup Domain разрешён
    if (!(PWR->CR & PWR_CR_DBP)) {
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_DBP;
    }

    // Проверяем, включён ли RTC (бит RTCEN в BDCR)
    if (!(RCC->BDCR & RCC_BDCR_RTCEN)) {
        // робуем запустить LSE
        RCC->BDCR |= RCC_BDCR_LSEON;
        timeout = RTC_TIMEOUT * 2;
        while (!(RCC->BDCR & RCC_BDCR_LSERDY) && timeout--) {
            __NOP();
        }
        if (RCC->BDCR & RCC_BDCR_LSERDY) {
            // LSE готов
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
    }

    // Ждём синхронизации регистров RTC (RSF)
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
    // Для LSE 32768 Гц получим 1 Гц на счётчике секунд
    RTC->PRER = (127UL << 16) | 255UL;

    // Установка 24-часового формата (FMT = 0)
    RTC->CR &= ~RTC_CR_FMT;

    // Выход из режима инициализации
    RTC->ISR &= ~RTC_ISR_INIT;

    // Ждём повторной синхронизации (опционально)
    timeout = RTC_TIMEOUT;
    while (!(RTC->ISR & RTC_ISR_RSF) && timeout--) {
        __NOP();
    }
}
