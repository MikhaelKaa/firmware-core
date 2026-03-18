/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

#ifndef DEV_W25Q_H
#define DEV_W25Q_H

#include <stddef.h>
#include <stdint.h>
#include "drv_face.h"

// PA15 w25q cs
// PB4  w25q do
// PB3  w25q clk
// PB5  w25q di

// Device-specific ioctl commands (base INTERFACE_CMD_DEVICE +)
#define W25Q_SET_ADDRESS    (INTERFACE_CMD_DEVICE + 0)  // arg: uint32_t* address
#define W25Q_GET_SIZE       (INTERFACE_CMD_DEVICE + 1)  // arg: uint32_t* size
#define W25Q_READ_ID        (INTERFACE_CMD_DEVICE + 2)  // arg: uint8_t id[3] (JEDEC)
#define W25Q_SECTOR_ERASE   (INTERFACE_CMD_DEVICE + 3)  // arg: uint32_t* sector_addr (or NULL for current)
#define W25Q_CHIP_ERASE     (INTERFACE_CMD_DEVICE + 4)  // arg: NULL

const drv_face_t* dev_w25q_get(void);

#endif /* DEV_W25Q_H */