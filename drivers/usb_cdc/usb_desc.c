/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

#include "usb_desc.h"

// Device Descriptor
const uint8_t usb_device_desc[18] = {
    18,                 // bLength
    0x01,               // bDescriptorType (Device)
    0x00, 0x02,         // bcdUSB (USB 2.0)
    0x02,               // bDeviceClass (CDC)
    0x00,               // bDeviceSubClass
    0x00,               // bDeviceProtocol
    64,                 // bMaxPacketSize0
    0x83, 0x04,         // idVendor (0x0483 - STMicroelectronics)
    0x40, 0x57,         // idProduct (0x5740)
    0x00, 0x02,         // bcdDevice (2.0)
    1,                  // iManufacturer
    2,                  // iProduct
    3,                  // iSerialNumber
    1                   // bNumConfigurations
};

// Configuration Descriptor + CDC Interfaces
const uint8_t usb_config_desc[67] = {
    // Configuration Descriptor
    9,                  // bLength
    0x02,               // bDescriptorType (Configuration)
    67, 0,              // wTotalLength
    2,                  // bNumInterfaces
    1,                  // bConfigurationValue
    0,                  // iConfiguration
    0x80,               // bmAttributes (bus powered)
    250,                // bMaxPower (500mA)
    
    // Interface 0: CDC Control
    9,                  // bLength
    0x04,               // bDescriptorType (Interface)
    0,                  // bInterfaceNumber
    0,                  // bAlternateSetting
    1,                  // bNumEndpoints
    0x02,               // bInterfaceClass (CDC)
    0x02,               // bInterfaceSubClass (ACM)
    0x01,               // bInterfaceProtocol (AT commands)
    0,                  // iInterface
    
    // CDC Header Functional Descriptor
    5,                  // bLength
    0x24,               // bDescriptorType (CS_INTERFACE)
    0x00,               // bDescriptorSubtype (Header)
    0x10, 0x01,         // bcdCDC (1.10)
    
    // CDC Call Management Functional Descriptor
    5,                  // bLength
    0x24,               // bDescriptorType (CS_INTERFACE)
    0x01,               // bDescriptorSubtype (Call Management)
    0x00,               // bmCapabilities
    1,                  // bDataInterface
    
    // CDC ACM Functional Descriptor
    4,                  // bLength
    0x24,               // bDescriptorType (CS_INTERFACE)
    0x02,               // bDescriptorSubtype (ACM)
    0x02,               // bmCapabilities
    
    // CDC Union Functional Descriptor
    5,                  // bLength
    0x24,               // bDescriptorType (CS_INTERFACE)
    0x06,               // bDescriptorSubtype (Union)
    0,                  // bControlInterface
    1,                  // bSubordinateInterface0
    
    // Endpoint: IN3 (Interrupt)
    7,                  // bLength
    0x05,               // bDescriptorType (Endpoint)
    0x83,               // bEndpointAddress (IN3)
    0x03,               // bmAttributes (Interrupt)
    8, 0,               // wMaxPacketSize
    16,                 // bInterval
    
    // Interface 1: CDC Data
    9,                  // bLength
    0x04,               // bDescriptorType (Interface)
    1,                  // bInterfaceNumber
    0,                  // bAlternateSetting
    2,                  // bNumEndpoints
    0x0A,               // bInterfaceClass (CDC Data)
    0x00,               // bInterfaceSubClass
    0x00,               // bInterfaceProtocol
    0,                  // iInterface
    
    // Endpoint: OUT1 (Bulk)
    7,                  // bLength
    0x05,               // bDescriptorType (Endpoint)
    0x01,               // bEndpointAddress (OUT1)
    0x02,               // bmAttributes (Bulk)
    64, 0,              // wMaxPacketSize
    0,                  // bInterval
    
    // Endpoint: IN1 (Bulk)
    7,                  // bLength
    0x05,               // bDescriptorType (Endpoint)
    0x81,               // bEndpointAddress (IN1)
    0x02,               // bmAttributes (Bulk)
    64, 0,              // wMaxPacketSize
    0                   // bInterval
};

// String Descriptors
static const uint8_t usb_string_lang[4] = {4, 0x03, 0x09, 0x04}; // English US
static const uint8_t usb_string_mfr[28] = {28, 0x03, 'M',0,'i',0,'c',0,'h',0,'a',0,'e',0,'l',0,' ',0,'K',0,'a',0,'a',0};
static const uint8_t usb_string_prod[32] = {32, 0x03, 'F',0,'i',0,'r',0,'m',0,'w',0,'a',0,'r',0,'e',0,' ',0,'C',0,'o',0,'r',0,'e',0};
static const uint8_t usb_string_serial[26] = {26, 0x03, '0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'1',0};

const uint8_t *usb_string_desc[] = {
    usb_string_lang,
    usb_string_mfr,
    usb_string_prod,
    usb_string_serial
};

const uint8_t usb_string_desc_count = 4;
