/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

#ifndef USB_DESC_H
#define USB_DESC_H

#include <stdint.h>

// USB Standard Requests
#define USB_REQ_GET_STATUS          0x00
#define USB_REQ_CLEAR_FEATURE       0x01
#define USB_REQ_SET_FEATURE         0x03
#define USB_REQ_SET_ADDRESS         0x05
#define USB_REQ_GET_DESCRIPTOR      0x06
#define USB_REQ_SET_DESCRIPTOR      0x07
#define USB_REQ_GET_CONFIGURATION   0x08
#define USB_REQ_SET_CONFIGURATION   0x09
#define USB_REQ_GET_INTERFACE       0x0A
#define USB_REQ_SET_INTERFACE       0x0B

// USB Descriptor Types
#define USB_DESC_TYPE_DEVICE        0x01
#define USB_DESC_TYPE_CONFIGURATION 0x02
#define USB_DESC_TYPE_STRING        0x03
#define USB_DESC_TYPE_INTERFACE     0x04
#define USB_DESC_TYPE_ENDPOINT      0x05

// CDC Class Requests
#define CDC_SET_LINE_CODING         0x20
#define CDC_GET_LINE_CODING         0x21
#define CDC_SET_CONTROL_LINE_STATE  0x22

extern const uint8_t usb_device_desc[18];
extern const uint8_t usb_config_desc[67];
extern const uint8_t *usb_string_desc[];
extern const uint8_t usb_string_desc_count;

#endif /* USB_DESC_H */
