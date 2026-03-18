/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#include <stdio.h>
#include <string.h>

#include "drv_face.h"
#include "svccalls.h"
#include "mem.h"
#include "ucmd.h"
#include "rtc.h"
#include "rtc_time.h"
#include "micros.h"
#include "pwm_led.h"
#include "w25q.h"

extern const drv_face_t dev_uart1;

int main(void)
{
    // char msg[] = "Its work";
    char *uart_ver = 0;
    drv_face_t* uart = 0;

    us_init();

    drv_face_t* led = dev_pwm_led_get();
    led->ioctl(INTERFACE_INIT, NULL);
    void (*led_proc)(void) = NULL;
    led->ioctl(INTERFACE_GET_PROC, &led_proc);

    // set driver over SVC
    fc_drv_table_set(&dev_uart1, 0);
    
    // Get driver over SVC
    fc_drv_table_get(&uart, 0);

    uart->ioctl(INTERFACE_INIT, NULL);

    // print uart version.
    uart->ioctl(INTERFACE_GET_INFO, &uart_ver);
    printf("%s\r\n", uart_ver);

    printf("RTC is initialized: %s\r\n", (RTC_is_initialized())?("true"):("false"));
    RTC_init();
    rtc_date_time_t date_time;
    RTC_get_date_time(&date_time);
    printf("time: %02d:%02d:%02d.%02d\r\n", date_time.hours, date_time.minutes, date_time.seconds, date_time.centiseconds);

    dev_memory_print_info();
    
    printf("micros: %ld\r\n", micros());
    
    // Установка режима дыхания
    pwm_led_set_mode(led, LED_MODE_BREATHE, 2000, 10, 200);

    drv_face_t* w25q =  dev_w25q_get();
    w25q->ioctl(INTERFACE_INIT, NULL);

    ucmd_default_init();

    while (1)
    {
        ucmd_default_proc();
        led_proc();
        for(volatile unsigned int i = 0; i < 1234U; i++) asm("nop");
    }
}
