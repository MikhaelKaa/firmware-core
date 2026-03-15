#include "ucmd.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "rtc_time.h"
#include "micros.h"

void time_set(uint8_t h, uint8_t m, uint8_t s);
void time_print(void);

int ucmd_time(int argc, char *argv[]) {
    static uint16_t h = 0, m = 0, s = 0;

    switch (argc) {
    case 1:
        time_print();
        return 0;

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
}

// Печать текущего времени с тремя знаками после запятой
void time_print(void) {
    // Читаем текущее время и дату
    rtc_date_time_t td;// = {0};
    RTC_get_date_time(&td);
    printf("%02d:%02d:%02d.%02d\r\n", td.hours, td.minutes, td.seconds, td.centiseconds);
    printf("micros: %ld\r\n", micros());
}
