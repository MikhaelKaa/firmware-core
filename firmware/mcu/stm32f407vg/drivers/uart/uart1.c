/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

#include <string.h>
#include <errno.h>

#include "uart.h"
#include "stm32f407xx.h"

#ifndef UART1_TX_TIMEOUT
#define UART1_TX_TIMEOUT (10000000U)
#endif // UART1_TX_TIMEOUT

// Buffer sizes
#ifndef UART1_TX_BUFFER_SIZE
#define UART1_TX_BUFFER_SIZE (256U)
#endif // UART1_TX_BUFFER_SIZE

#ifndef UART1_RX_BUFFER_SIZE
#define UART1_RX_BUFFER_SIZE (256U)
#endif // UART1_RX_BUFFER_SIZE

// Driver version
const char *dev_uart1_version = "stm32f407vgt6 uart1; PA9-TX, PA10-RX; 115200n1; hardcode ver 0.0.0";

// Static buffers
static uint8_t tx_buffer[UART1_TX_BUFFER_SIZE];
static volatile uint8_t rx_buffer[UART1_RX_BUFFER_SIZE];

// Ring buffer pointers for RX
static volatile uint32_t rx_read_pos = 0;
static volatile uint32_t rx_write_pos = 0;

// DMA transfer state
static volatile uint8_t tx_in_progress = 0;
static volatile uint32_t tx_complete_flag = 0;

// Callback for RX idle detection
static void (*uart1_idle_callback)(void) = NULL;


static int uart_available(void);

// Open UART (interface implementation)
static int uart_init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    
    // Configure GPIO for USART1 (PA9 - TX, PA10 - RX)
    GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
    GPIOA->MODER |= (2 << GPIO_MODER_MODER9_Pos) | (2 << GPIO_MODER_MODER10_Pos);
    
    // Alternate function AF7 for USART1
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10);
    GPIOA->AFR[1] |= (7 << (4 * 1)) | (7 << (4 * 2));
    
    // High speed
    GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED9_Pos) | (3 << GPIO_OSPEEDR_OSPEED10_Pos);
    
    // Configure USART1 - 115200 baud at 84MHz
    USART1->BRR = (84000000 + 115200 / 2) / 115200; //TODO: Variable baudrate, CLK
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_IDLEIE; // Added IDLEIE
    USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
    USART1->CR1 |= USART_CR1_UE;
    
    // Configure DMA for transmission (USART1_TX -> DMA2 Stream7)
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream7->CR & DMA_SxCR_EN);
    
    DMA2_Stream7->PAR = (uint32_t)&USART1->DR;
    DMA2_Stream7->M0AR = (uint32_t)tx_buffer;
    DMA2_Stream7->NDTR = 0;
    
    DMA2_Stream7->CR = (4 << DMA_SxCR_CHSEL_Pos) |  // Channel 4
                       DMA_SxCR_MINC |              // Memory increment
                       DMA_SxCR_DIR_0 |             // Memory to peripheral
                       DMA_SxCR_TCIE;               // Transfer complete interrupt
    
    // Configure DMA for reception (USART1_RX -> DMA2 Stream5) - Circular mode
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream5->CR & DMA_SxCR_EN);
    
    DMA2_Stream5->PAR = (uint32_t)&USART1->DR;
    DMA2_Stream5->M0AR = (uint32_t)rx_buffer;
    DMA2_Stream5->NDTR = UART1_RX_BUFFER_SIZE;
    
    DMA2_Stream5->CR = (4 << DMA_SxCR_CHSEL_Pos) |  // Channel 4
                       DMA_SxCR_MINC |              // Memory increment
                       DMA_SxCR_CIRC |              // Circular mode
                       DMA_SxCR_HTIE |              // Half transfer interrupt
                       DMA_SxCR_TCIE;               // Transfer complete interrupt
    
    // Enable DMA reception
    DMA2_Stream5->CR |= DMA_SxCR_EN;
    
    // Enable interrupts
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    
    // Clear buffers
    memset((void *)tx_buffer, 0, UART1_TX_BUFFER_SIZE);
    memset((void *)rx_buffer, 0, UART1_RX_BUFFER_SIZE);
    return 0;  // Success
}

// Close UART (interface implementation)
static int uart_deinit(void) {
    // Disable UART and DMA
    USART1->CR1 &= ~USART_CR1_UE;
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    
    // Disable interrupts
    NVIC_DisableIRQ(USART1_IRQn);
    NVIC_DisableIRQ(DMA2_Stream7_IRQn);
    NVIC_DisableIRQ(DMA2_Stream5_IRQn);
    
    return 0;  // Success
}

// Write data to UART (interface implementation)
static int uart_write(const void *buf, size_t count) {
    if (buf == NULL || count == 0) {
        return -EINVAL;
    }
    
    if (count > UART1_TX_BUFFER_SIZE) {
        count = UART1_TX_BUFFER_SIZE;
    }

    // Wait for previous transmission to complete
    uint32_t timeout = UART1_TX_TIMEOUT;  // Timeout counter
    while (tx_in_progress && timeout--) {
        __asm__("nop");
    }
    
    if (timeout == 0) {
        return -ETIMEDOUT;
    }
    
    // Copy data to buffer
    memcpy(tx_buffer, buf, count);
    tx_in_progress = 1;
    
    // Configure and start DMA transfer
    DMA2_Stream7->M0AR = (uint32_t)tx_buffer;
    DMA2_Stream7->NDTR = count;
    DMA2_Stream7->CR |= DMA_SxCR_EN;
    
    return (int)count;
}

// Read data from UART (interface implementation)
static int uart_read(void *buf, size_t count) {
    if (buf == NULL) {
        return -EINVAL;
    }
    
    uint8_t *buffer = (uint8_t *)buf;
    size_t bytes_read = 0;
    
    // Calculate available bytes in ring buffer
    int available = uart_available();
    if (available == 0) {
        return 0;  // No data available
    }
    
    if (count > (size_t)available) {
        count = (size_t)available;
    }
    
    // Read data from ring buffer
    for (size_t i = 0; i < count; i++) {
        buffer[i] = rx_buffer[rx_read_pos];
        rx_read_pos = (rx_read_pos + 1) % UART1_RX_BUFFER_SIZE;
        bytes_read++;
    }
    
    return (int)bytes_read;
}

// Check how many bytes are available to read
static int uart_available(void) {
    uint32_t current_ndtr = DMA2_Stream5->NDTR;
    uint32_t available_bytes = (UART1_RX_BUFFER_SIZE - current_ndtr - rx_read_pos) % UART1_RX_BUFFER_SIZE;
    return (int)available_bytes;
}

// IO Control for UART (interface implementation)
static int uart_ioctl(int cmd, void *arg) {
    switch (cmd) {
        case INTERFACE_INIT:
            uart_init();
            return 0;

        case INTERFACE_DEINIT:
            uart_deinit();
            return 0;

        case UART_GET_AVAILABLE:
            if (arg != NULL) {
                *(int *)arg = uart_available();
            }
            return 0;

        case INTERFACE_GET_INFO:
            if (arg != NULL) {
                *(const char **)arg = dev_uart1_version;
                return 0;
            }
            return -EINVAL;

        case UART_SET_RX_IDLE_CALLBACK:
            {
                // Используем union для преобразования void* в указатель на функцию
                union {
                    void (*func)(void);
                    void *ptr;
                } cast;
                cast.ptr = arg;
                uart1_idle_callback = cast.func;
            }
            return 0;

        default:
            return -ENOTSUP;  // Command not supported
    }
}

// UART device instance
const drv_face_t dev_uart1 = {
    .read = uart_read, 
    .write = uart_write, 
    .ioctl = uart_ioctl
};

const drv_face_t* dev_uart1_get(void)
{
    return (const drv_face_t*) &dev_uart1;
}

// USART1 Interrupt Handler
void USART1_IRQHandler(void) {
    uint32_t sr = USART1->SR;
    
    // RXNE interrupt - data received (handled by DMA)
    if (sr & USART_SR_RXNE) {
        (void)USART1->DR;  // Read to clear flag
    }
    
    // IDLE line detected - end of packet
    if (sr & USART_SR_IDLE) {
        (void)USART1->DR;  // Read to clear IDLE flag
        if (uart1_idle_callback) uart1_idle_callback();
    }
}

// DMA2 Stream7 Interrupt Handler (Transmission)
void DMA2_Stream7_IRQHandler(void) {
    if (DMA2->HISR & DMA_HISR_TCIF7) {
        DMA2->HIFCR |= DMA_HIFCR_CTCIF7;  // Clear transfer complete flag
        tx_in_progress = 0;  // Mark as ready for next transmission
    }
}

// DMA2 Stream5 Interrupt Handler (Reception)
void DMA2_Stream5_IRQHandler(void) {
    // Half transfer complete
    if (DMA2->HISR & DMA_HISR_HTIF5) {
        DMA2->HIFCR |= DMA_HIFCR_CHTIF5;
        // Optional: handle half buffer event
    }
    
    // Transfer complete
    if (DMA2->HISR & DMA_HISR_TCIF5) {
        DMA2->HIFCR |= DMA_HIFCR_CTCIF5;
        // Optional: handle full buffer event
    }
}