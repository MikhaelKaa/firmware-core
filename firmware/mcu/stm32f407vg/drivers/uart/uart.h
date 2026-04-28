/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

#ifndef DEV_UART1_H
#define DEV_UART1_H

#include <stddef.h>
#include <stdint.h>
#include "drv_face.h"

// UART-specific ioctl commands
#define UART_GET_AVAILABLE          (INTERFACE_CMD_DEVICE + 0)
#define UART_SET_RX_IDLE_CALLBACK   (INTERFACE_CMD_DEVICE + 1)
#define UART_SET_BAUDRATE           (INTERFACE_CMD_DEVICE + 2)
#define UART_TX_READY               (INTERFACE_CMD_DEVICE + 3) ///< arg=NULL, returns 1 if TX DMA is free

const drv_face_t* dev_uart1_get(void);
const drv_face_t* dev_uart2_get(void);

#endif /* DEV_UART1_H */