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
    uint32_t multi_packet;
    uint16_t last_total;
    uint16_t last_sent;
    uint16_t just_sent;
    uint16_t last_wLength;
    uint32_t xfrc_with_state1;
    uint32_t get_config_desc;
    uint8_t last_setup[8];
} usb_cdc_debug_stats_t;

const drv_face_t* dev_usb_cdc_get(void);

#endif /* DEV_USB_CDC_H */
