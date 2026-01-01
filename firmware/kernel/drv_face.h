/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#ifndef _DRV_FACE_H
#define _DRV_FACE_H

#include <stddef.h>
#include <stdint.h>
#include <errno.h>
#include <stdio.h>

/* Common ioctl commands */
#define INTERFACE_GET_INFO   0x1000 /* Get device info */
#define INTERFACE_INIT       0x1001 /* Init device */
#define INTERFACE_DEINIT     0x1002 /* Deinit device */

/* Device-specific command space */
#define INTERFACE_CMD_DEVICE 0x2000 /* Base for device-specific commands */


/**
 * struct drv_face - Unified device interface
 * @read:  Read data from device
 * @write: Write data to device
 * @ioctl: Device control and configuration
 *
 * Generic interface for all device types in the system.
 * Functions should return 0 on success or negative errno on error.
 */
typedef struct drv_face
{
    int (*read)(void* buf, size_t len);
    int (*write)(const void* buf, size_t len);
    int (*ioctl)(int cmd, void* arg);
} drv_face_t;

/// @brief 
/// @param dev 
/// @param pos 
/// @return 
ssize_t drv_table_set(const drv_face_t* dev, unsigned int pos);

/// @brief 
/// @param dev 
/// @param pos 
/// @return 
ssize_t drv_table_get(drv_face_t** dev, unsigned int pos);

#endif /* _DRV_FACE_H */