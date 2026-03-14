/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */
// RTC time/date functions implementation

#include "rtc_time.h"
#include "rtc.h"
#include "stm32f407xx.h"

#define RTC_TIMEOUT         1000000U

// Преобразование байта в BCD (0-99 -> 0x00-0x99)
static inline uint8_t bin2bcd(uint8_t bin)
{
    return (uint8_t)(((bin / 10U) << 4U) | (bin % 10U));
}

// Преобразование BCD в байт (0x00-0x99 -> 0-99)
static inline uint8_t bcd2bin(uint8_t bcd)
{
    return (uint8_t)(((bcd >> 4U) * 10U) + (bcd & 0x0FU));
}

// Снятие защиты записи RTC (последовательность 0xCA, 0x53)
static inline void rtc_write_protection_disable(void)
{
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
}

// Восстановление защиты записи RTC (любое значение, кроме 0xCA и 0x53)
static inline void rtc_write_protection_enable(void)
{
    RTC->WPR = 0xFF;
}

uint8_t RTC_is_initialized(void)
{
    return (RTC->BKP0R == rtc_magic_number) ? 1U : 0U;
}

void RTC_set_date_time(const rtc_date_time_t *dt)
{
    uint32_t tr, dr;
    uint32_t timeout;

    // Доступ к Backup Domain
    if (!(PWR->CR & PWR_CR_DBP)) {
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_DBP;
    }

    // Ждём синхронизации перед входом в INIT
    timeout = RTC_TIMEOUT;
    while (!(RTC->ISR & RTC_ISR_RSF) && timeout--) {
        __NOP();
    }

    // Снимаем защиту записи
    rtc_write_protection_disable();

    // Вход в режим инициализации
    RTC->ISR |= RTC_ISR_INIT;
    timeout = RTC_TIMEOUT;
    while (!(RTC->ISR & RTC_ISR_INITF) && timeout--) {
        __NOP();
    }

    // Формирование регистра времени
    tr = ((uint32_t)bin2bcd(dt->hours)   << 16U) |
         ((uint32_t)bin2bcd(dt->minutes) << 8U)  |
         ((uint32_t)bin2bcd(dt->seconds));

    // Формирование регистра даты
    dr = ((uint32_t)bin2bcd(dt->year)   << 16U) |
         ((uint32_t)(dt->weekday & 0x7U) << 13U) |
         ((uint32_t)bin2bcd(dt->month)  << 8U)  |
         ((uint32_t)bin2bcd(dt->day));

    RTC->TR = tr;
    RTC->DR = dr;

    // Выход из режима инициализации
    RTC->ISR &= ~RTC_ISR_INIT;

    // Восстанавливаем защиту записи
    rtc_write_protection_enable();

    // Ждём синхронизации
    timeout = RTC_TIMEOUT;
    while (!(RTC->ISR & RTC_ISR_RSF) && timeout--) {
        __NOP();
    }
}

void RTC_get_date_time(rtc_date_time_t *dt)
{
    uint32_t tr, dr;
    uint32_t timeout;

    // Ждём синхронизации (теневые регистры)
    timeout = RTC_TIMEOUT;
    while (!(RTC->ISR & RTC_ISR_RSF) && timeout--) {
        __NOP();
    }

    tr = RTC->TR;
    dr = RTC->DR;

    dt->seconds = bcd2bin((tr >> 0U)  & 0x7FU);
    dt->minutes = bcd2bin((tr >> 8U)  & 0x7FU);
    dt->hours   = bcd2bin((tr >> 16U) & 0x3FU);

    dt->day     = bcd2bin((dr >> 0U)  & 0x3FU);
    dt->month   = bcd2bin((dr >> 8U)  & 0x1FU);
    dt->year    = bcd2bin((dr >> 16U) & 0xFFU);
    dt->weekday = (uint8_t)((dr >> 13U) & 0x7U);
}