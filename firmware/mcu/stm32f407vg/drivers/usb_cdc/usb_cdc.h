/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

#ifndef DEV_USB_CDC_H
#define DEV_USB_CDC_H

#include <stddef.h>
#include <stdint.h>
#include "drv_face.h"

// USB CDC-specific ioctl commands
#define USB_CDC_GET_AVAILABLE       (INTERFACE_CMD_DEVICE + 0)
#define USB_CDC_SET_RX_CALLBACK     (INTERFACE_CMD_DEVICE + 1)
#define USB_CDC_GET_DTR             (INTERFACE_CMD_DEVICE + 2)
#define USB_CDC_GET_RTS             (INTERFACE_CMD_DEVICE + 3)
#define USB_CDC_GET_DEBUG_STATS     (INTERFACE_CMD_DEVICE + 4)

typedef struct {
    uint32_t reset_count;
    uint32_t setup_count;
    uint32_t rxflvl_count;
    uint32_t address_set;
    uint32_t ep0_in_xfrc;
    uint8_t last_setup[8];
} usb_cdc_debug_stats_t;

const drv_face_t* dev_usb_cdc_get(void);

#endif /* DEV_USB_CDC_H */
