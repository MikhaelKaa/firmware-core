/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#include <stdio.h>
#include <string.h>

#include "drv_face.h"
#include "svccalls.h"
#include "mem.h"
#include "ucmd.h"
#include "rtc.h"


extern const drv_face_t dev_uart1;

int main(void)
{
    char msg[] = "Its work";
    char *uart_ver = 0;
    drv_face_t* uart = 0;

    // set driver over SVC
    fc_drv_table_set(&dev_uart1, 0);
    
    // Get driver over SVC
    fc_drv_table_get(&uart, 0);

    uart->ioctl(INTERFACE_INIT, NULL);

    // print uart version.
    uart->ioctl(INTERFACE_GET_INFO, &uart_ver);
    printf("%s\r\n", uart_ver);

    RTC_init();

    dev_memory_print_info();
    
    // for(volatile int i = 0; i < INT16_MAX*256; i++) asm("nop");
    ucmd_default_init();

    while (1)
    {
        ucmd_default_proc();

        for(volatile unsigned int i = 0; i < 12345U; i++) asm("nop");

    }
}
