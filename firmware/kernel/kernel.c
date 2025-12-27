/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#include <stdio.h>
#include <string.h>

#include "dev_interface.h"

extern const interface_t dev_uart1;

int main(void)
{
    char msg[] = "Its work";
    char *uart_ver = 0;

    dev_uart1.ioctl(INTERFACE_INIT, NULL);

    // print uart version.
    dev_uart1.ioctl(INTERFACE_GET_INFO, &uart_ver);
    printf("%s\r\n", uart_ver);
    for(volatile int i = 0; i < INT16_MAX*256; i++) asm("nop");

    while (1)
    {
        printf("%s\r\n", msg);
        for(volatile int i = 0; i < INT16_MAX*64; i++) asm("nop");
    }
}
