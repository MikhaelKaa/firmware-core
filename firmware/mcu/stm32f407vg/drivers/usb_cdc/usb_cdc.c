/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

#include <string.h>
#include <errno.h>

#include "usb_cdc.h"
#include "usb_desc.h"
#include "stm32f407xx.h"

#define USBx USB_OTG_FS

// USB Device state
typedef enum {
    USB_STATE_DEFAULT = 0,
    USB_STATE_ADDRESSED,
    USB_STATE_CONFIGURED
} usb_device_state_t;

static usb_device_state_t usb_state = USB_STATE_DEFAULT;

// Buffer sizes
#ifndef USB_CDC_TX_BUFFER_SIZE
#define USB_CDC_TX_BUFFER_SIZE (512U)
#endif

#ifndef USB_CDC_RX_BUFFER_SIZE
#define USB_CDC_RX_BUFFER_SIZE (512U)
#endif

// USB CDC Endpoints
#define CDC_CMD_EP      0x82  // EP2 IN
#define CDC_DATA_IN_EP  0x81  // EP1 IN
#define CDC_DATA_OUT_EP 0x01  // EP1 OUT

// Driver version
const char *dev_usb_cdc_version = "stm32f407vgt6 usb_cdc; PA11-DM, PA12-DP; bare-metal ver 0.0.1";

// Static buffers
static uint8_t tx_buffer[USB_CDC_TX_BUFFER_SIZE];
static volatile uint8_t rx_buffer[USB_CDC_RX_BUFFER_SIZE];

// Ring buffer pointers for RX
static volatile uint32_t rx_read_pos = 0;
static volatile uint32_t rx_write_pos = 0;

// Transfer state
static volatile uint8_t tx_in_progress = 0;
static volatile uint8_t usb_configured = 0;

// Line coding (baudrate, stop bits, parity, data bits)
static struct {
    uint32_t bitrate;
    uint8_t format;    // 0: 1 stop bit, 1: 1.5 stop bits, 2: 2 stop bits
    uint8_t parity;    // 0: None, 1: Odd, 2: Even
    uint8_t data_bits; // 5, 6, 7, 8, 16
} line_coding = {115200, 0, 0, 8};

// Control line state (DTR, RTS)
static volatile uint16_t control_line_state = 0;

// Callback for RX data
static void (*usb_cdc_rx_callback)(void) = NULL;

static int usb_cdc_available(void);

// Initialize USB peripheral
static int usb_cdc_init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    
    // Configure GPIO for USB OTG FS (PA11 - DM, PA12 - DP)
    GPIOA->MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12);
    GPIOA->MODER |= (2 << GPIO_MODER_MODER11_Pos) | (2 << GPIO_MODER_MODER12_Pos);
    
    // Alternate function AF10 for USB OTG FS
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL11 | GPIO_AFRH_AFSEL12);
    GPIOA->AFR[1] |= (10 << (4 * 3)) | (10 << (4 * 4));
    
    // Very high speed
    GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED11_Pos) | (3 << GPIO_OSPEEDR_OSPEED12_Pos);
    
    // No pull-up/pull-down
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD11 | GPIO_PUPDR_PUPD12);
    
    // TODO: Full USB OTG FS initialization
    // This requires implementing USB core init, device init, endpoint setup
    // For now, this is a skeleton that compiles but doesn't function
    
    // Enable USB interrupt
    NVIC_EnableIRQ(OTG_FS_IRQn);
    
    // Clear buffers
    memset((void *)tx_buffer, 0, USB_CDC_TX_BUFFER_SIZE);
    memset((void *)rx_buffer, 0, USB_CDC_RX_BUFFER_SIZE);
    
    usb_configured = 0;
    usb_state = USB_STATE_DEFAULT;
    
    return 0;
}

// Deinitialize USB
static int usb_cdc_deinit(void) {
    // Disable USB
    NVIC_DisableIRQ(OTG_FS_IRQn);
    RCC->AHB2ENR &= ~RCC_AHB2ENR_OTGFSEN;
    
    usb_configured = 0;
    return 0;
}

// Write data to USB CDC
static int usb_cdc_write(const void *buf, size_t count) {
    if (buf == NULL || count == 0) {
        return -EINVAL;
    }
    
    if (!usb_configured) {
        return -ENOTCONN;
    }
    
    if (count > USB_CDC_TX_BUFFER_SIZE) {
        count = USB_CDC_TX_BUFFER_SIZE;
    }

    // Wait for previous transmission to complete
    uint32_t timeout = 10000000;
    while (tx_in_progress && timeout--) {
        __asm__("nop");
    }
    
    if (timeout == 0) {
        return -ETIMEDOUT;
    }
    
    // Copy data to buffer
    memcpy(tx_buffer, buf, count);
    tx_in_progress = 1;
    
    // TODO: Trigger USB transmission
    // This is a placeholder - actual USB endpoint write needed
    
    tx_in_progress = 0; // Temporary
    
    return (int)count;
}

// Read data from USB CDC
static int usb_cdc_read(void *buf, size_t count) {
    if (buf == NULL) {
        return -EINVAL;
    }
    
    uint8_t *buffer = (uint8_t *)buf;
    size_t bytes_read = 0;
    
    int available = usb_cdc_available();
    if (available == 0) {
        return 0;
    }
    
    if (count > (size_t)available) {
        count = (size_t)available;
    }
    
    // Read data from ring buffer
    for (size_t i = 0; i < count; i++) {
        buffer[i] = rx_buffer[rx_read_pos];
        rx_read_pos = (rx_read_pos + 1) % USB_CDC_RX_BUFFER_SIZE;
        bytes_read++;
    }
    
    return (int)bytes_read;
}

// Check how many bytes are available to read
static int usb_cdc_available(void) {
    int available = (int)((rx_write_pos - rx_read_pos + USB_CDC_RX_BUFFER_SIZE) % USB_CDC_RX_BUFFER_SIZE);
    return available;
}

// IO Control for USB CDC
static int usb_cdc_ioctl(int cmd, void *arg) {
    switch (cmd) {
        case INTERFACE_INIT:
            return usb_cdc_init();

        case INTERFACE_DEINIT:
            return usb_cdc_deinit();

        case USB_CDC_GET_AVAILABLE:
            if (arg != NULL) {
                *(int *)arg = usb_cdc_available();
            }
            return 0;

        case INTERFACE_GET_INFO:
            if (arg != NULL) {
                *(const char **)arg = dev_usb_cdc_version;
                return 0;
            }
            return -EINVAL;

        case USB_CDC_SET_RX_CALLBACK:
            {
                union {
                    void (*func)(void);
                    void *ptr;
                } cast;
                cast.ptr = arg;
                usb_cdc_rx_callback = cast.func;
            }
            return 0;

        case USB_CDC_GET_DTR:
            if (arg != NULL) {
                *(uint8_t *)arg = (control_line_state & 0x01) ? 1 : 0;
            }
            return 0;

        case USB_CDC_GET_RTS:
            if (arg != NULL) {
                *(uint8_t *)arg = (control_line_state & 0x02) ? 1 : 0;
            }
            return 0;

        default:
            return -ENOTSUP;
    }
}

// USB CDC device instance
const drv_face_t dev_usb_cdc = {
    .read = usb_cdc_read, 
    .write = usb_cdc_write, 
    .ioctl = usb_cdc_ioctl
};

const drv_face_t* dev_usb_cdc_get(void)
{
    return (const drv_face_t*) &dev_usb_cdc;
}

// USB OTG FS Interrupt Handler
void OTG_FS_IRQHandler(void) {
    // TODO: Full USB interrupt handling
    // This is a skeleton implementation
    // Requires:
    // 1. USB Reset handling
    // 2. Enumeration (SETUP packet processing)
    // 3. CDC class requests (SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE)
    // 4. Bulk IN/OUT endpoint handling for data transfer
    // 5. Ring buffer management for RX/TX
    
    // For now, just clear pending interrupts
    uint32_t gintsts = USBx->GINTSTS & USBx->GINTMSK;
    
    if (gintsts & USB_OTG_GINTSTS_USBRST) {
        USBx->GINTSTS = USB_OTG_GINTSTS_USBRST;
        usb_state = USB_STATE_DEFAULT;
        usb_configured = 0;
    }
    
    if (gintsts & USB_OTG_GINTSTS_ENUMDNE) {
        USBx->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
    }
    
    if (gintsts & USB_OTG_GINTSTS_RXFLVL) {
        // RX FIFO handling
    }
    
    if (gintsts & USB_OTG_GINTSTS_IEPINT) {
        // IN endpoint handling
        tx_in_progress = 0;
    }
    
    if (gintsts & USB_OTG_GINTSTS_OEPINT) {
        // OUT endpoint handling
        if (usb_cdc_rx_callback) {
            usb_cdc_rx_callback();
        }
    }
}
