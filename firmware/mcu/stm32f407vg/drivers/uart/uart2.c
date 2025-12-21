/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#include <string.h>
#include <errno.h>

#include "uart.h"
#include "stm32f407xx.h"

#ifndef UART2_TX_TIMEOUT
#define UART2_TX_TIMEOUT (10000000U)
#endif // UART2_TX_TIMEOUT

// Buffer sizes
#ifndef UART2_TX_BUFFER_SIZE
#define UART2_TX_BUFFER_SIZE (256U)
#endif // UART2_TX_BUFFER_SIZE

#ifndef UART2_RX_BUFFER_SIZE
#define UART2_RX_BUFFER_SIZE (256U)
#endif // UART2_RX_BUFFER_SIZE

// Static buffers
static uint8_t tx_buffer[UART2_TX_BUFFER_SIZE];
static volatile uint8_t rx_buffer[UART2_RX_BUFFER_SIZE];

// Ring buffer pointers for RX
static volatile uint32_t rx_read_pos = 0;
static volatile uint32_t rx_write_pos = 0;

// DMA transfer state
static volatile uint8_t tx_in_progress = 0;
static volatile uint32_t tx_complete_flag = 0;

// Driver version
const char *dev_uart2_version = "1.0.0";

static int uart_available(void);

// Open UART2 (interface implementation)
static int uart_init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    
    // Configure GPIO for USART2 (PA2 - TX, PA3 - RX)
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= (2 << GPIO_MODER_MODER2_Pos) | (2 << GPIO_MODER_MODER3_Pos);
    
    // Alternate function AF7 for USART2
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
    GPIOA->AFR[0] |= (7 << (4 * 2)) | (7 << (4 * 3));
    
    // High speed
    GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED2_Pos) | (3 << GPIO_OSPEEDR_OSPEED3_Pos);
    
    // Configure USART2 - 115200 baud at 42MHz (APB1 = 42MHz)
    USART2->BRR = (42000000 + 115200 / 2) / 115200;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART2->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
    USART2->CR1 |= USART_CR1_UE;
    
    // Configure DMA for transmission (USART2_TX -> DMA1 Stream6, Channel 4)
    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream6->CR & DMA_SxCR_EN);
    
    DMA1_Stream6->PAR = (uint32_t)&USART2->DR;
    DMA1_Stream6->M0AR = (uint32_t)tx_buffer;
    DMA1_Stream6->NDTR = 0;
    
    DMA1_Stream6->CR = (4 << DMA_SxCR_CHSEL_Pos) |  // Channel 4
                       DMA_SxCR_MINC |              // Memory increment
                       DMA_SxCR_DIR_0 |             // Memory to peripheral
                       DMA_SxCR_TCIE;               // Transfer complete interrupt
    
    // Configure DMA for reception (USART2_RX -> DMA1 Stream5, Channel 4) - Circular mode
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN);
    
    DMA1_Stream5->PAR = (uint32_t)&USART2->DR;
    DMA1_Stream5->M0AR = (uint32_t)rx_buffer;
    DMA1_Stream5->NDTR = UART2_RX_BUFFER_SIZE;
    
    DMA1_Stream5->CR = (4 << DMA_SxCR_CHSEL_Pos) |  // Channel 4
                       DMA_SxCR_MINC |              // Memory increment
                       DMA_SxCR_CIRC |              // Circular mode
                       DMA_SxCR_HTIE |              // Half transfer interrupt
                       DMA_SxCR_TCIE;               // Transfer complete interrupt
    
    // Enable DMA reception
    DMA1_Stream5->CR |= DMA_SxCR_EN;
    
    // Enable interrupts
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    
    // Clear buffers
    memset((void *)tx_buffer, 0, UART2_TX_BUFFER_SIZE);
    memset((void *)rx_buffer, 0, UART2_RX_BUFFER_SIZE);
    return 0;  // Success
}

// Close UART2 (interface implementation)
static int uart_deinit(void) {
    // Disable UART and DMA
    USART2->CR1 &= ~USART_CR1_UE;
    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    
    // Disable interrupts
    NVIC_DisableIRQ(USART2_IRQn);
    NVIC_DisableIRQ(DMA1_Stream6_IRQn);
    NVIC_DisableIRQ(DMA1_Stream5_IRQn);
    
    return 0;  // Success
}

// Write data to UART2 (interface implementation)
static int uart_write(const void *buf, size_t count) {
    if (buf == NULL || count == 0) {
        return -EINVAL;
    }
    
    if (count > UART2_TX_BUFFER_SIZE) {
        count = UART2_TX_BUFFER_SIZE;
    }

    // Wait for previous transmission to complete
    uint32_t timeout = UART2_TX_TIMEOUT;
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
    DMA1_Stream6->M0AR = (uint32_t)tx_buffer;
    DMA1_Stream6->NDTR = count;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
    
    return (int)count;
}

// Read data from UART2 (interface implementation)
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
        rx_read_pos = (rx_read_pos + 1) % UART2_RX_BUFFER_SIZE;
        bytes_read++;
    }
    
    return (int)bytes_read;
}

// Check how many bytes are available to read (FIXED VERSION)
static int uart_available(void) {
    uint32_t current_ndtr = DMA1_Stream5->NDTR;
    uint32_t available_bytes = (UART2_RX_BUFFER_SIZE - current_ndtr - rx_read_pos) % UART2_RX_BUFFER_SIZE;
    return (int)available_bytes;
}

// TODO: WTF???
// Flush RX buffer
static int uart_flush(void) {
    // rx_read_pos = (UART2_RX_BUFFER_SIZE - DMA1_Stream5->NDTR) % UART2_RX_BUFFER_SIZE;
    return 0;
}

// IO Control for UART2 (interface implementation)
static int uart_ioctl(int cmd, void *arg) {
    switch (cmd) {
        case UART_INIT:
            uart_init();
            return 0;

        case UART_DEINIT:
            uart_deinit();
            return 0;
        case UART_GET_AVAILABLE:
            if (arg != NULL) {
                *(int *)arg = uart_available();
            }
            return 0;
            
        case UART_GET_VERSION:
            if (arg != NULL) {
                *(const char **)arg = dev_uart2_version;
                return 0;
            }
            return -EINVAL;       
            
        case UART_FLUSH:
            return uart_flush();
            
        default:
            return -ENOTSUP;  // Command not supported
    }
}

// UART2 device instance
static const interface_t dev_uart2 = {.read = uart_read, .write = uart_write, .ioctl = uart_ioctl};

const interface_t* dev_uart2_get(void)
{
    return (const interface_t*)&dev_uart2;
}

// USART2 Interrupt Handler
void USART2_IRQHandler(void) {
    // RXNE interrupt - data received
    if (USART2->SR & USART_SR_RXNE) {
        volatile uint8_t data = (uint8_t)USART2->DR;  // Read to clear flag
        (void)data;  // Suppress unused warning
        // Data is handled by DMA, this is just for flag clearing
    }
}

// DMA1 Stream6 Interrupt Handler (Transmission)
void DMA1_Stream6_IRQHandler(void) {
    if (DMA1->HISR & DMA_HISR_TCIF6) {
        DMA1->HIFCR |= DMA_HIFCR_CTCIF6;  // Clear transfer complete flag
        tx_in_progress = 0;  // Mark as ready for next transmission
    }
}

// DMA1 Stream5 Interrupt Handler (Reception)
void DMA1_Stream5_IRQHandler(void) {
    // Half transfer complete
    if (DMA1->HISR & DMA_HISR_HTIF5) {
        DMA1->HIFCR |= DMA_HIFCR_CHTIF5;
        // Optional: handle half buffer event
    }
    
    // Transfer complete
    if (DMA1->HISR & DMA_HISR_TCIF5) {
        DMA1->HIFCR |= DMA_HIFCR_CTCIF5;
        // Optional: handle full buffer event
    }
}