/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */
// Функции для работы с временем и датой RTC

#ifndef RTC_TIME_H
#define RTC_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f407xx.h"
#include "rtc.h"

// Структура для хранения даты и времени
typedef struct {
    uint8_t year;         // 0-99 (год от 2000)
    uint8_t month;        // 1-12
    uint8_t weekday;      // 1-7 (понедельник=1)
    uint8_t day;          // 1-31
    uint8_t hours;        // 0-23
    uint8_t minutes;      // 0-59
    uint8_t seconds;      // 0-59
    uint8_t centiseconds; // 0-99 (сотые доли секунды)
} rtc_date_time_t;

// Проверка инициализации RTC
uint8_t RTC_is_initialized(void);

// Установка даты и времени в RTC
void RTC_set_date_time(const rtc_date_time_t *dt);

// Получение текущей даты и времени из RTC
void RTC_get_date_time(rtc_date_time_t *dt);

#ifdef __cplusplus
}
#endif

#endif /* RTC_TIME_H */
