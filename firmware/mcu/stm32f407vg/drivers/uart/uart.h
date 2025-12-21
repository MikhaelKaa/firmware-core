/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#ifndef DEV_UART1_H
#define DEV_UART1_H

#include <stddef.h>
#include <stdint.h>
#include "dev_interface.h"

// UART-specific ioctl commands
#define UART_INIT           (INTERFACE_CMD_DEVICE + 0)
#define UART_DEINIT         (INTERFACE_CMD_DEVICE + 1)
#define UART_GET_AVAILABLE  (INTERFACE_CMD_DEVICE + 2)
#define UART_GET_VERSION    (INTERFACE_CMD_DEVICE + 3)
#define UART_FLUSH          (INTERFACE_CMD_DEVICE + 4)

const interface_t* dev_uart1_get(void);
const interface_t* dev_uart2_get(void);

#endif /* DEV_UART1_H */