/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#ifndef DEV_UART1_H
#define DEV_UART1_H

#include <stddef.h>
#include <stdint.h>
#include "drv_face.h"

// UART-specific ioctl commands
#define UART_GET_AVAILABLE  (INTERFACE_CMD_DEVICE + 0)

// const drv_face_t dev_uart1;

const drv_face_t* dev_uart1_get(void);
const drv_face_t* dev_uart2_get(void);

#endif /* DEV_UART1_H */