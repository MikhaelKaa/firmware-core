#include "ucmd.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

// Подключаем наши CMSIS-модули RTC
#include "rtc_time.h"
#include "rtc.h"          // для RTC_init(), если вызывается отдельно

// Глобальные переменные для хранения текущего времени и долей секунды
static rtc_date_time_t rtc_now;
__attribute__((aligned(4))) static float subseconds = 0.0f;

void time_set(uint8_t h, uint8_t m, uint8_t s);
void time_print(void);
void time_update(void);

int ucmd_time(int argc, char *argv[]) {
    static uint16_t h = 0, m = 0, s = 0;
    switch (argc) {
    case 1:
        time_print();
        return 0;

    case 2:
        if (strcmp(&argv[1][0], "u") == 0 ||
            strcmp(&argv[1][0], "update") == 0) {
            time_update();
            return 0;
        }
        break;

    case 4:
        sscanf(&argv[1][0], "%hu", &h);
        sscanf(&argv[2][0], "%hu", &m);
        sscanf(&argv[3][0], "%hu", &s);
        time_set((uint8_t)h, (uint8_t)m, (uint8_t)s);
        printf("set new time %u:%u:%u\r\n", h, m, s);
        time_print();
        return 0;

    default:
        return UCMD_CMD_NOT_FOUND;
    }
    return -1;
}

// Установка нового времени (дата остаётся текущей)
void time_set(uint8_t h, uint8_t m, uint8_t s) {
    // Читаем текущую дату из RTC, чтобы не потерять её
    rtc_date_time_t new_dt;
    RTC_get_date_time(&new_dt);

    // Обновляем время
    new_dt.hours   = h;
    new_dt.minutes = m;
    new_dt.seconds = s;

    // Записываем новые значения в RTC
    RTC_set_date_time(&new_dt);

    // Обновляем глобальные переменные
    time_update();
}

// Обновление глобальных переменных из RTC и вычисление долей секунды
void time_update(void) {
    // Читаем текущее время и дату
    RTC_get_date_time(&rtc_now);

    // TODO: not work, hardfault float
    // Потом починю.
    // Быстро читаем субсекунды через inline-функцию
    // uint32_t ssr = RTC_SUBSEC;

    // Вычисляем дробную часть секунды:
    // SSR уменьшается от 255 до 0, поэтому (255 - SSR) / 255
    // subseconds = (255.0f - (float)ssr) / 255.0f;
}

// Печать текущего времени с тремя знаками после запятой
void time_print(void) {
    time_update();
    // float total_seconds = (float)rtc_now.seconds + subseconds;
    // printf("%02d:%02d:%2.3f\r\n", rtc_now.hours, rtc_now.minutes, total_seconds);
    printf("%02d:%02d:%02d\r\n", rtc_now.hours, rtc_now.minutes, rtc_now.seconds);
    
}