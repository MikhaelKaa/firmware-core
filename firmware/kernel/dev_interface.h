/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#ifndef _INTERFACE_H
#define _INTERFACE_H

#include <stddef.h>
#include <stdint.h>
#include <errno.h>


/**
 * struct interface - Unified device interface
 * @read:  Read data from device
 * @write: Write data to device
 * @ioctl: Device control and configuration
 *
 * Generic interface for all device types in the system.
 * Functions should return 0 on success or negative errno on error.
 */
typedef struct interface
{
    int (*read)(void* buf, size_t len);
    int (*write)(const void* buf, size_t len);
    int (*ioctl)(int cmd, void* arg);
} interface_t;

/* Common ioctl commands */
#define INTERFACE_GET_INFO   0x1000 /* Get device info */
#define INTERFACE_GET_STATUS 0x1001 /* Get device status */
#define INTERFACE_RESET      0x1002 /* Reset device */
#define INTERFACE_SET_CONFIG 0x1003 /* Set device configuration */
#define INTERFACE_GET_CONFIG 0x1004 /* Get device configuration */

/* Device-specific command space */
#define INTERFACE_CMD_DEVICE 0x2000 /* Base for device-specific commands */

#endif /* _INTERFACE_H */