/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

#include <string.h>
#include <errno.h>
#include <stdio.h>

#include "usb_cdc.h"
#include "usb_desc.h"
#include "stm32f407xx.h"

#define USBx USB_OTG_FS
#define USBx_DEVICE ((USB_OTG_DeviceTypeDef *)((uint32_t)USBx + USB_OTG_DEVICE_BASE))
#define USBx_INEP(i) ((USB_OTG_INEndpointTypeDef *)((uint32_t)USBx + USB_OTG_IN_ENDPOINT_BASE + ((i) * 0x20)))
#define USBx_OUTEP(i) ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)USBx + USB_OTG_OUT_ENDPOINT_BASE + ((i) * 0x20)))

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

// Control line state (DTR, RTS)
static volatile uint16_t control_line_state = 0;

// Callback for RX data
static void (*usb_cdc_rx_callback)(void) = NULL;

// SETUP packet buffer
static uint8_t setup_packet[8];

// EP0 state
static uint8_t ep0_state = 0; // 0=IDLE, 1=DATA_IN, 2=DATA_OUT, 3=STATUS_IN, 4=STATUS_OUT
static const uint8_t *ep0_tx_ptr = NULL;
static uint16_t ep0_tx_len = 0;
static uint16_t ep0_tx_total = 0;  // Total bytes to send
static uint16_t ep0_tx_sent = 0;   // Bytes already sent

// Open endpoint
static void usb_ep_open(uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps) {
    uint8_t ep_num = ep_addr & 0x7F;
    uint8_t is_in = (ep_addr & 0x80) != 0;
    
    if (is_in) {
        USBx_DEVICE->DAINTMSK |= (1 << ep_num);
        USBx_INEP(ep_num)->DIEPCTL = (ep_mps & 0x7FF) | 
                                      (ep_type << 18) | 
                                      (ep_num << 22) |
                                      USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                                      USB_OTG_DIEPCTL_USBAEP;
    } else {
        USBx_DEVICE->DAINTMSK |= (1 << (ep_num + 16));
        USBx_OUTEP(ep_num)->DOEPCTL = (ep_mps & 0x7FF) | 
                                       (ep_type << 18) |
                                       USB_OTG_DOEPCTL_SD0PID_SEVNFRM |
                                       USB_OTG_DOEPCTL_USBAEP;
    }
}

// Write to TX FIFO
static void usb_write_fifo(uint8_t ep_num, const uint8_t *src, uint16_t len) {
    uint32_t count32 = (len + 3) / 4;
    __IO uint32_t *fifo = (__IO uint32_t *)((uint32_t)USBx + 0x1000 * (ep_num + 1));
    
    for (uint32_t i = 0; i < count32; i++) {
        uint32_t data = 0;
        for (int j = 0; j < 4 && (i * 4 + j) < len; j++) {
            data |= ((uint32_t)src[i * 4 + j]) << (j * 8);
        }
        *fifo = data;
    }
}

// Read from RX FIFO
static void usb_read_fifo(uint8_t *dest, uint16_t len) {
    uint32_t count32 = (len + 3) / 4;
    __IO uint32_t *fifo = (__IO uint32_t *)((uint32_t)USBx + 0x1000);
    
    for (uint32_t i = 0; i < count32; i++) {
        uint32_t data = *fifo;
        for (int j = 0; j < 4 && (i * 4 + j) < len; j++) {
            dest[i * 4 + j] = (data >> (j * 8)) & 0xFF;
        }
    }
}

// EP0 transmit
static void usb_ep0_transmit(const uint8_t *data, uint16_t len) {
    // Don't start new transmission if one is in progress
    if (ep0_state == 1) {
        return;
    }
    
    uint16_t pkt = (len > 64) ? 64 : len;
    
    ep0_tx_ptr = (uint8_t *)data;
    ep0_tx_total = len;
    ep0_tx_sent = 0;
    ep0_state = 1;
    
    USBx_INEP(0)->DIEPTSIZ = (1 << 19) | pkt;
    USBx_INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
    
    if (pkt > 0) {
        usb_write_fifo(0, data, pkt);
    }
}

// Handle standard device requests
static void usb_handle_setup(void) {
    // Don't reset EP0 state if transmission in progress
    if (ep0_state != 1) {
        ep0_state = 0;
    }
    ep0_tx_ptr = NULL;
    ep0_tx_len = 0;
    
    uint8_t req_type = setup_packet[0];
    uint8_t req = setup_packet[1];
    uint16_t wValue = setup_packet[2] | (setup_packet[3] << 8);
    uint16_t wLength = setup_packet[6] | (setup_packet[7] << 8);
    
    if ((req_type & 0x60) == 0x00) { // Standard request
        if (req == USB_REQ_GET_DESCRIPTOR) {
            uint8_t desc_type = wValue >> 8;
            uint8_t desc_index = wValue & 0xFF;
            
            if (desc_type == USB_DESC_TYPE_DEVICE) {
                usb_ep0_transmit(usb_device_desc, (wLength < 18) ? wLength : 18);
            } else if (desc_type == USB_DESC_TYPE_CONFIGURATION) {
                uint16_t len = (wLength < 67) ? wLength : 67;
                usb_ep0_transmit(usb_config_desc, len);
            } else if (desc_type == USB_DESC_TYPE_STRING) {
                if (desc_index < usb_string_desc_count) {
                    uint8_t len = usb_string_desc[desc_index][0];
                    usb_ep0_transmit(usb_string_desc[desc_index], (wLength < len) ? wLength : len);
                }
            } else if (desc_type == 0x06) { // DEVICE_QUALIFIER - not supported for FS-only device
                // STALL EP0
                USBx_INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
            }
        } else if (req == USB_REQ_SET_ADDRESS) {
            uint8_t addr = wValue & 0x7F;
            // Set address immediately (before status stage)
            USBx_DEVICE->DCFG = (USBx_DEVICE->DCFG & ~USB_OTG_DCFG_DAD) | (addr << 4);
            usb_state = (addr != 0) ? USB_STATE_ADDRESSED : USB_STATE_DEFAULT;
            usb_ep0_transmit(NULL, 0); // Status stage
        } else if (req == USB_REQ_SET_CONFIGURATION) {
            usb_configured = 1;
            usb_state = USB_STATE_CONFIGURED;
            usb_ep0_transmit(NULL, 0); // Status stage
        } else {
            usb_ep0_transmit(NULL, 0); // ACK
        }
    } else if ((req_type & 0x60) == 0x20) { // Class request
        if (req == CDC_SET_LINE_CODING) {
            // Will receive 7 bytes in DATA stage
            ep0_state = 2; // DATA_OUT
            USBx_OUTEP(0)->DOEPTSIZ = (1 << 19) | 7;
            USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
        } else if (req == CDC_GET_LINE_CODING) {
            uint8_t coding[7] = {0x00, 0xC2, 0x01, 0x00, 0, 0, 8}; // 115200, 1 stop, no parity, 8 bits
            usb_ep0_transmit(coding, 7);
        } else if (req == CDC_SET_CONTROL_LINE_STATE) {
            control_line_state = wValue;
            usb_ep0_transmit(NULL, 0); // ACK
        } else {
            usb_ep0_transmit(NULL, 0); // ACK
        }
    }
}

static int usb_cdc_available(void);

// USB soft disconnect/reconnect via PC9 (D+ pull-up control)
static void usb_soft_disconnect(void) {
    GPIOC->BSRR = GPIO_BSRR_BR9;  // Reset PC9 (disable pull-up)
}

static void usb_soft_reconnect(void) {
    GPIOC->BSRR = GPIO_BSRR_BS9;  // Set PC9 (enable pull-up)
}

// USB Core initialization
static void usb_core_init(void) {
    // Core soft reset
    USBx->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    while (USBx->GRSTCTL & USB_OTG_GRSTCTL_CSRST);
    
    // Wait for AHB idle
    while (!(USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));
    
    // Deactivate power down and disable VBUS sensing
    USBx->GCCFG = USB_OTG_GCCFG_PWRDWN | USB_OTG_GCCFG_NOVBUSSENS;
    
    // Force device mode, select embedded FS PHY
    USBx->GUSBCFG = USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL;
    
    // USB turnaround time for 48MHz
    USBx->GUSBCFG &= ~(0xFUL << 10);
    USBx->GUSBCFG |= (0x6 << 10);
    
    // Wait 50ms for mode switch
    for (volatile int i = 0; i < 500000; i++);
}

// USB Device initialization
static void usb_device_init(void) {
    // Soft disconnect first
    USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
    
    // Device speed: Full Speed
    USBx_DEVICE->DCFG |= USB_OTG_DCFG_DSPD;
    
    // Flush FIFOs
    USBx->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (0x10 << 6);
    while (USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);
    
    USBx->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
    while (USBx->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH);
    
    // Clear interrupts
    USBx_DEVICE->DIEPMSK = 0;
    USBx_DEVICE->DOEPMSK = 0;
    USBx_DEVICE->DAINT = 0xFFFFFFFF;
    USBx_DEVICE->DAINTMSK = 0;
    
    for (int i = 0; i < 4; i++) {
        USBx_INEP(i)->DIEPINT = 0xFF;
        USBx_OUTEP(i)->DOEPINT = 0xFF;
    }
    
    // Configure FIFOs
    // RX FIFO: 128 words
    USBx->GRXFSIZ = 128;
    
    // TX0 FIFO: 64 words at offset 128
    USBx->DIEPTXF0_HNPTXFSIZ = (64 << 16) | 128;
    
    // TX1 FIFO: 64 words at offset 192
    USBx->DIEPTXF[0] = (64 << 16) | 192;
    
    // TX2 FIFO: 16 words at offset 256 (for EP3 interrupt)
    USBx->DIEPTXF[1] = (16 << 16) | 256;
    
    // Enable interrupts
    USBx->GINTMSK = USB_OTG_GINTMSK_USBRST | 
                    USB_OTG_GINTMSK_ENUMDNEM |
                    USB_OTG_GINTMSK_RXFLVLM |
                    USB_OTG_GINTMSK_IEPINT |
                    USB_OTG_GINTMSK_OEPINT;
    
    // Enable global interrupt
    USBx->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
    
    // Clear global interrupt status
    USBx->GINTSTS = 0xFFFFFFFF;
    
    // Soft connect (enable pull-up on D+)
    USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;
}

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
    
    // PC9 as output for D+ pull-up control (1.5k to D+)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~GPIO_MODER_MODER9;
    GPIOC->MODER |= (1 << GPIO_MODER_MODER9_Pos);  // Output
    GPIOC->BSRR = GPIO_BSRR_BS9;  // Set high (enable pull-up)
    
    // Initialize USB core and device
    usb_core_init();
    usb_device_init();
    
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
    
    // Start transmission on EP1 IN
    USBx_INEP(1)->DIEPTSIZ = (1 << 19) | count;
    USBx_INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
    
    usb_write_fifo(1, tx_buffer, count);
    
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
    
    // Disable USB interrupt during read to prevent race condition
    NVIC_DisableIRQ(OTG_FS_IRQn);
    
    // Read data from ring buffer
    for (size_t i = 0; i < count; i++) {
        buffer[i] = rx_buffer[rx_read_pos];
        rx_read_pos = (rx_read_pos + 1) % USB_CDC_RX_BUFFER_SIZE;
        bytes_read++;
    }
    
    NVIC_EnableIRQ(OTG_FS_IRQn);
    
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

        case USB_CDC_SOFT_DISCONNECT:
            usb_soft_disconnect();
            return 0;

        case USB_CDC_SOFT_RECONNECT:
            usb_soft_reconnect();
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
    uint32_t gintsts = USBx->GINTSTS;
    
    // USB Reset
    if (gintsts & USB_OTG_GINTSTS_USBRST) {
        USBx->GINTSTS = USB_OTG_GINTSTS_USBRST;
        
        // Reset device state
        usb_state = USB_STATE_DEFAULT;
        usb_configured = 0;
        ep0_state = 0;
        
        // Clear all interrupts
        USBx->GINTSTS = 0xFFFFFFFF;
        
        // Set device address to 0
        USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;
        
        // Flush FIFOs
        USBx->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (0x10 << 6);
        while (USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);
        
        USBx->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
        while (USBx->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH);
        
        // Enable endpoint interrupts for EP0, EP1
        USBx_DEVICE->DAINTMSK = 0x30003; // EP0 IN/OUT, EP1 IN/OUT, EP3 IN
        USBx_DEVICE->DOEPMSK = USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM;
        USBx_DEVICE->DIEPMSK = USB_OTG_DIEPMSK_XFRCM;
        
        // Open EP0
        USBx_INEP(0)->DIEPCTL = USB_OTG_DIEPCTL_USBAEP | USB_OTG_DIEPCTL_SNAK;
        USBx_OUTEP(0)->DOEPCTL = USB_OTG_DOEPCTL_USBAEP;
        
        // Prepare EP0 OUT for SETUP
        USBx_OUTEP(0)->DOEPTSIZ = (3 << 29) | (1 << 19) | 64;
        USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
        
        // Open EP1 IN (Bulk)
        USBx_INEP(1)->DIEPCTL = USB_OTG_DIEPCTL_SNAK |
                                 (1 << 22) | // TX FIFO 1
                                 (2 << 18) | // Bulk
                                 USB_OTG_DIEPCTL_USBAEP |
                                 64;
        
        // Open EP1 OUT (Bulk)
        USBx_OUTEP(1)->DOEPCTL = USB_OTG_DOEPCTL_CNAK |
                                  (2 << 18) | // Bulk
                                  USB_OTG_DOEPCTL_USBAEP |
                                  64;
        
        USBx_OUTEP(1)->DOEPTSIZ = (1 << 19) | 64;
        USBx_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA;
        
        // Open EP3 IN (Interrupt)
        USBx_INEP(3)->DIEPCTL = USB_OTG_DIEPCTL_SNAK |
                                 (2 << 22) | // TX FIFO 2
                                 (3 << 18) | // Interrupt
                                 USB_OTG_DIEPCTL_USBAEP |
                                 8;
    }
    
    // Enumeration done
    if (gintsts & USB_OTG_GINTSTS_ENUMDNE) {
        USBx->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
        
        // Set EP0 max packet size to 64
        USBx_INEP(0)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;
        USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;
        return;
    }
    
    // RX FIFO non-empty - MUST be processed first
    if (gintsts & USB_OTG_GINTSTS_RXFLVL) {
        uint32_t grxsts = USBx->GRXSTSP;
        uint8_t epnum = grxsts & 0xF;
        uint16_t count = (grxsts >> 4) & 0x7FF;
        uint8_t pktsts = (grxsts >> 17) & 0xF;
        
        if (pktsts == 6 && epnum == 0) { // SETUP packet data in FIFO
            usb_read_fifo(setup_packet, 8);
            usb_handle_setup(); // Handle immediately
        } else if (pktsts == 2 && count > 0) { // OUT packet
            if (epnum == 1) {
                // EP1 OUT - CDC data
                uint8_t temp[64];
                usb_read_fifo(temp, count);
                
                // Copy to ring buffer with overflow check
                for (uint16_t i = 0; i < count; i++) {
                    uint32_t next_pos = (rx_write_pos + 1) % USB_CDC_RX_BUFFER_SIZE;
                    // Check for buffer overflow
                    if (next_pos != rx_read_pos) {
                        rx_buffer[rx_write_pos] = temp[i];
                        rx_write_pos = next_pos;
                    }
                    // If buffer full, drop data silently
                }
                
                if (usb_cdc_rx_callback) {
                    usb_cdc_rx_callback();
                }
            } else if (epnum == 0) {
                // EP0 OUT - control data (e.g., line coding)
                uint8_t temp[64];
                usb_read_fifo(temp, count);
            }
        } else if (pktsts == 4) { // OUT transfer completed
            if (epnum == 1) {
                // Re-enable EP1 OUT for next packet
                USBx_OUTEP(1)->DOEPTSIZ = (1 << 19) | 64;
                USBx_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
            }
        }
        // Don't return - continue to process OEPINT
    }
    
    // OUT endpoint interrupt - process before IN
    if (gintsts & USB_OTG_GINTSTS_OEPINT) {
        USBx->GINTSTS = USB_OTG_GINTSTS_OEPINT;
        uint32_t ep_intr = (USBx_DEVICE->DAINT >> 16) & 0xFFFF;
        
        if (ep_intr & 0x1) { // EP0 OUT
            uint32_t doepint = USBx_OUTEP(0)->DOEPINT;
            
            if (doepint & USB_OTG_DOEPINT_STUP) {
                // SETUP packet received
                usb_handle_setup();
                USBx_OUTEP(0)->DOEPINT = USB_OTG_DOEPINT_STUP;
                // Prepare for next SETUP
                USBx_OUTEP(0)->DOEPTSIZ = (3 << 29) | (1 << 19) | 64;
                USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
            }
            
            if (doepint & USB_OTG_DOEPINT_XFRC) {
                USBx_OUTEP(0)->DOEPINT = USB_OTG_DOEPINT_XFRC;
                // Prepare EP0 OUT for next SETUP
                USBx_OUTEP(0)->DOEPTSIZ = (3 << 29) | (1 << 19) | 64;
                USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
            }
        }
        
        if (ep_intr & 0x2) { // EP1 OUT
            uint32_t doepint = USBx_OUTEP(1)->DOEPINT;
            
            if (doepint & USB_OTG_DOEPINT_XFRC) {
                USBx_OUTEP(1)->DOEPINT = USB_OTG_DOEPINT_XFRC;
                
                // Re-enable for next reception
                USBx_OUTEP(1)->DOEPTSIZ = (1 << 19) | 64;
                USBx_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
            }
        }
        return;
    }
    
    // IN endpoint interrupt
    if (gintsts & USB_OTG_GINTSTS_IEPINT) {
        USBx->GINTSTS = USB_OTG_GINTSTS_IEPINT;
        uint32_t ep_intr = USBx_DEVICE->DAINT & 0xFFFF;
        
        if (ep_intr & 0x1) { // EP0 IN
            uint32_t diepint = USBx_INEP(0)->DIEPINT;
            
            if (diepint & USB_OTG_DIEPINT_XFRC) {
                USBx_INEP(0)->DIEPINT = USB_OTG_DIEPINT_XFRC;
                
                // Update pointer after sending
                if (ep0_state == 1) {
                    uint16_t just_sent = (ep0_tx_total - ep0_tx_sent > 64) ? 64 : (ep0_tx_total - ep0_tx_sent);
                    ep0_tx_sent += just_sent;
                    uint16_t remaining = ep0_tx_total - ep0_tx_sent;
                    
                    if (remaining > 0) {
                        // Send next chunk
                        uint16_t pkt = (remaining > 64) ? 64 : remaining;
                        
                        USBx_INEP(0)->DIEPTSIZ = (1 << 19) | pkt;
                        USBx_INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
                        usb_write_fifo(0, ep0_tx_ptr + ep0_tx_sent, pkt);
                    } else {
                        ep0_state = 0;
                        // Prepare EP0 OUT
                        USBx_OUTEP(0)->DOEPTSIZ = (3 << 29) | (1 << 19) | 64;
                        USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
                    }
                } else {
                    ep0_state = 0;
                    // Prepare EP0 OUT for next SETUP
                    USBx_OUTEP(0)->DOEPTSIZ = (3 << 29) | (1 << 19) | 64;
                    USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
                }
            }
        }
        
        if (ep_intr & 0x2) { // EP1 IN
            uint32_t diepint = USBx_INEP(1)->DIEPINT;
            
            if (diepint & USB_OTG_DIEPINT_XFRC) {
                USBx_INEP(1)->DIEPINT = USB_OTG_DIEPINT_XFRC;
                tx_in_progress = 0;
            }
        }
        return;
    }
}
