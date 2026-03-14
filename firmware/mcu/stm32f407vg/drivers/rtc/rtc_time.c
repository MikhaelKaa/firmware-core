/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */
// RTC time/date functions implementation

#include "rtc_time.h"
#include "stm32f407xx.h"


#define RTC_TIMEOUT         1000000U

// Преобразование байта в BCD (0-99 -> 0x00-0x99)
static inline uint8_t bin2bcd(uint8_t bin)
{
    return ((bin / 10) << 4) | (bin % 10);
}

// Преобразование BCD в байт (0x00-0x99 -> 0-99)
static inline uint8_t bcd2bin(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

uint8_t RTC_is_initialized(void)
{
    return ((RTC->BKP0R & 0xFFFF) == RTC_MAGIC_NUMBER) ? 1 : 0;
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

    // if (!(RCC->BDCR & RCC_BDCR_RTCEN)) {
    //     RCC->BDCR |= RCC_BDCR_RTCEN;
    //     // после включения нужно подождать синхронизации
    //     timeout = RTC_TIMEOUT;
    //     while (!(RTC->ISR & RTC_ISR_RSF) && timeout--) __NOP();
    // }

    // Вход в режим инициализации
    RTC->ISR |= RTC_ISR_INIT;
    timeout = RTC_TIMEOUT;
    while (!(RTC->ISR & RTC_ISR_INITF) && timeout--) {
        __NOP();
    }

    // Формирование регистра времени
    tr = (bin2bcd(dt->hours)   << 16) |
         (bin2bcd(dt->minutes) << 8)  |
         (bin2bcd(dt->seconds));

    // Формирование регистра даты
    dr = ((uint32_t)bin2bcd(dt->year)   << 16) |
         ((uint32_t)(dt->weekday & 0x7) << 13) |
         ((uint32_t)bin2bcd(dt->month)  << 8)  |
         (bin2bcd(dt->day));

    RTC->TR = tr;
    RTC->DR = dr;

    // Выход из инициализации
    RTC->ISR &= ~RTC_ISR_INIT;

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

    dt->seconds = bcd2bin((tr >> 0)  & 0x7F);
    dt->minutes = bcd2bin((tr >> 8)  & 0x7F);
    dt->hours   = bcd2bin((tr >> 16) & 0x3F);

    dt->day     = bcd2bin((dr >> 0)  & 0x3F);
    dt->month   = bcd2bin((dr >> 8)  & 0x1F);
    dt->year    = bcd2bin((dr >> 16) & 0xFF);
    dt->weekday = (dr >> 13) & 0x7;
}