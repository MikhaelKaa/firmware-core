/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#include <string.h>
#include <errno.h>
#include "w25q.h"
#include "stm32f407xx.h"

// ----------------------------------------------------------------------------
// Configuration constants
// ----------------------------------------------------------------------------
#ifndef W25Q_SPI
#define W25Q_SPI            SPI1
#endif

#ifndef W25Q_CS_PORT
#define W25Q_CS_PORT        GPIOA               // CS on PA15
#endif

#ifndef W25Q_CS_PIN
#define W25Q_CS_PIN         15                  // PA15
#endif

#ifndef W25Q_SCK_PIN
#define W25Q_SCK_PIN        3   // PB3
#endif

#ifndef W25Q_MISO_PIN
#define W25Q_MISO_PIN       4   // PB4
#endif

#ifndef W25Q_MOSI_PIN
#define W25Q_MOSI_PIN       5   // PB5
#endif

#ifndef W25Q_PAGE_SIZE
#define W25Q_PAGE_SIZE      256U
#endif

#ifndef W25Q_SECTOR_SIZE
#define W25Q_SECTOR_SIZE    4096U
#endif

#ifndef W25Q_FLASH_SIZE
#define W25Q_FLASH_SIZE     (16 * 1024 * 1024)   // 16 MB for W25Q128 (default)
#endif

#ifndef W25Q_TIMEOUT
#define W25Q_TIMEOUT        1000000U
#endif

// ----------------------------------------------------------------------------
// W25Q SPI Commands
// ----------------------------------------------------------------------------
#define CMD_WRITE_ENABLE    0x06
#define CMD_WRITE_DISABLE   0x04
#define CMD_READ_STATUS     0x05
#define CMD_READ_DATA       0x03
#define CMD_PAGE_PROGRAM    0x02
#define CMD_SECTOR_ERASE    0x20
#define CMD_CHIP_ERASE      0xC7
#define CMD_JEDEC_ID        0x9F

// Status register bits
#define SR_BUSY             0x01

// ----------------------------------------------------------------------------
// Driver version string
// ----------------------------------------------------------------------------
static const char *dev_w25q_version = "w25q; SPI1; CS PA15; auto-detect size; Winbond; ver 0.2.0";

// ----------------------------------------------------------------------------
// Static state
// ----------------------------------------------------------------------------
static uint32_t current_address = 0;
static uint32_t flash_size = W25Q_FLASH_SIZE;   // будет переопределено при инициализации
static uint8_t initialized = 0;

// ----------------------------------------------------------------------------
// Low-level SPI & GPIO helpers
// ----------------------------------------------------------------------------

#define W25Q_CS_LOW()   (W25Q_CS_PORT->BSRR = (1U << (W25Q_CS_PIN + 16U)))
#define W25Q_CS_HIGH()  (W25Q_CS_PORT->BSRR = (1U << W25Q_CS_PIN))

/**
 * @brief Transmit and receive one byte over SPI (blocking)
 * @param tx byte to send
 * @return received byte
 */
static uint8_t spi_transfer_byte(uint8_t tx) {
    while (!(W25Q_SPI->SR & SPI_SR_TXE));
    W25Q_SPI->DR = tx;
    while (!(W25Q_SPI->SR & SPI_SR_RXNE));
    return (uint8_t)W25Q_SPI->DR;
}

/**
 * @brief Write multiple bytes (no read)
 */
static void spi_write(const uint8_t *data, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        spi_transfer_byte(data[i]);
    }
}

/**
 * @brief Read multiple bytes (send 0xFF dummy)
 */
static void spi_read(uint8_t *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = spi_transfer_byte(0xFF);
    }
}

/**
 * @brief Wait until the W25Q internal operation completes (WIP bit cleared)
 * @return 0 on success, -ETIMEDOUT on timeout
 */
static int w25q_wait_busy(void) {
    uint32_t timeout = W25Q_TIMEOUT;
    W25Q_CS_LOW();
    spi_transfer_byte(CMD_READ_STATUS);
    while (timeout--) {
        uint8_t status = spi_transfer_byte(0xFF);
        if (!(status & SR_BUSY)) {
            W25Q_CS_HIGH();
            return 0;
        }
    }
    W25Q_CS_HIGH();
    return -ETIMEDOUT;
}

/**
 * @brief Send Write Enable command
 */
static void w25q_write_enable(void) {
    W25Q_CS_LOW();
    spi_transfer_byte(CMD_WRITE_ENABLE);
    W25Q_CS_HIGH();
}

/**
 * @brief Read JEDEC ID (3 bytes)
 */
static int w25q_read_jedec_id(uint8_t *id) {
    if (!id) return -EINVAL;
    W25Q_CS_LOW();
    spi_transfer_byte(CMD_JEDEC_ID);
    spi_read(id, 3);
    W25Q_CS_HIGH();
    return 0;
}

// ----------------------------------------------------------------------------
// Driver interface implementation
// ----------------------------------------------------------------------------

static int w25q_init(void) {
    if (initialized) return 0;

    // 1. Enable clocks for GPIOA, GPIOB and SPI1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // 2. Configure GPIO for SPI1 alternate function (AF5) on PB3, PB4, PB5
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOB->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1); // AF
    GPIOB->AFR[0] |= (5 << (4 * 3)) | (5 << (4 * 4)) | (5 << (4 * 5)); // AF5

    // 3. Configure PA15 as output (CS)
    GPIOA->MODER &= ~GPIO_MODER_MODER15;
    GPIOA->MODER |= GPIO_MODER_MODER15_0;   // Output
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_15;    // Push-pull
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED15; // High speed
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR15;    // No pull
    W25Q_CS_HIGH();                          // CS inactive by default

    // 4. Configure SPI1
    W25Q_SPI->CR1 &= ~SPI_CR1_SPE;           // Disable SPI
    W25Q_SPI->CR1 = SPI_CR1_MSTR | (4 << SPI_CR1_BR_Pos) |
                    SPI_CR1_SSM | SPI_CR1_SSI; // Software NSS, internal high
    W25Q_SPI->CR2 = 0;                        // No interrupts, DMA disabled
    W25Q_SPI->CR1 |= SPI_CR1_SPE;             // Enable SPI

    // 5. Read JEDEC ID to determine flash size
    uint8_t id[3];
    if (w25q_read_jedec_id(id) == 0) {
        // Winbond manufacturer ID = 0xEF, memory type often 0x40
        if (id[0] == 0xEF && id[1] == 0x40) {
            switch (id[2]) {
                case 0x14: flash_size = 1 * 1024 * 1024; break;   // W25Q80  (8 Mbit)
                case 0x15: flash_size = 2 * 1024 * 1024; break;   // W25Q16  (16 Mbit)
                case 0x16: flash_size = 4 * 1024 * 1024; break;   // W25Q32  (32 Mbit)
                case 0x17: flash_size = 8 * 1024 * 1024; break;   // W25Q64  (64 Mbit)
                case 0x18: flash_size = 16 * 1024 * 1024; break;  // W25Q128 (128 Mbit)
                case 0x19: flash_size = 32 * 1024 * 1024; break;  // W25Q256 (256 Mbit)
                default:
                    // Unknown capacity, keep default
                    flash_size = W25Q_FLASH_SIZE;
                    break;
            }
        } else {
            // Not Winbond or unknown type, keep default
            flash_size = W25Q_FLASH_SIZE;
        }
    } else {
        // Failed to read ID, keep default
        flash_size = W25Q_FLASH_SIZE;
    }

    initialized = 1;
    return 0;
}

static int w25q_deinit(void) {
    if (!initialized) return 0;

    W25Q_SPI->CR1 &= ~SPI_CR1_SPE;             // Disable SPI

    // Reset GPIO to analog mode (optional)
    GPIOB->MODER |= GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5;
    GPIOA->MODER |= GPIO_MODER_MODER15;

    initialized = 0;
    return 0;
}

static int w25q_read(void *buf, size_t count) {
    if (!initialized) return -EIO;
    if (!buf || count == 0) return -EINVAL;

    uint8_t *buffer = (uint8_t *)buf;
    uint32_t addr = current_address;
    size_t bytes_to_read = count;

    if (addr + bytes_to_read > flash_size) {
        bytes_to_read = flash_size - addr;
    }
    if (bytes_to_read == 0) return 0;

    W25Q_CS_LOW();
    spi_transfer_byte(CMD_READ_DATA);
    spi_transfer_byte((addr >> 16) & 0xFF);
    spi_transfer_byte((addr >> 8) & 0xFF);
    spi_transfer_byte(addr & 0xFF);
    spi_read(buffer, bytes_to_read);
    W25Q_CS_HIGH();

    current_address = addr + bytes_to_read;
    return (int)bytes_to_read;
}

static int w25q_write(const void *buf, size_t count) {
    if (!initialized) return -EIO;
    if (!buf || count == 0) return -EINVAL;

    const uint8_t *data = (const uint8_t *)buf;
    uint32_t addr = current_address;
    size_t total_written = 0;

    while (count > 0) {
        uint32_t page_offset = addr % W25Q_PAGE_SIZE;
        uint32_t page_remain = W25Q_PAGE_SIZE - page_offset;
        uint32_t chunk = (count < page_remain) ? count : page_remain;

        if (addr + chunk > flash_size) {
            chunk = flash_size - addr;
        }
        if (chunk == 0) break;

        w25q_write_enable();

        W25Q_CS_LOW();
        spi_transfer_byte(CMD_PAGE_PROGRAM);
        spi_transfer_byte((addr >> 16) & 0xFF);
        spi_transfer_byte((addr >> 8) & 0xFF);
        spi_transfer_byte(addr & 0xFF);
        spi_write(data, chunk);
        W25Q_CS_HIGH();

        if (w25q_wait_busy() != 0) {
            return -ETIMEDOUT;
        }

        addr += chunk;
        data += chunk;
        count -= chunk;
        total_written += chunk;
    }

    current_address = addr;
    return (int)total_written;
}

static int w25q_ioctl(int cmd, void *arg) {
    switch (cmd) {
        case INTERFACE_INIT:
            return w25q_init();

        case INTERFACE_DEINIT:
            return w25q_deinit();

        case INTERFACE_GET_INFO:
            if (arg) {
                *(const char **)arg = dev_w25q_version;
                return 0;
            }
            return -EINVAL;

        case W25Q_SET_ADDRESS:
            if (arg) {
                current_address = *(uint32_t *)arg;
                return 0;
            }
            return -EINVAL;

        case W25Q_GET_SIZE:
            if (arg) {
                *(uint32_t *)arg = flash_size;
                return 0;
            }
            return -EINVAL;

        case W25Q_READ_ID: {
            if (!arg) return -EINVAL;
            uint8_t *id = (uint8_t *)arg;
            W25Q_CS_LOW();
            spi_transfer_byte(CMD_JEDEC_ID);
            spi_read(id, 3);
            W25Q_CS_HIGH();
            return 0;
        }

        case W25Q_SECTOR_ERASE: {
            uint32_t sector_addr;
            if (arg) {
                sector_addr = *(uint32_t *)arg;
            } else {
                sector_addr = current_address;
            }
            sector_addr &= ~(W25Q_SECTOR_SIZE - 1);

            w25q_write_enable();
            W25Q_CS_LOW();
            spi_transfer_byte(CMD_SECTOR_ERASE);
            spi_transfer_byte((sector_addr >> 16) & 0xFF);
            spi_transfer_byte((sector_addr >> 8) & 0xFF);
            spi_transfer_byte(sector_addr & 0xFF);
            W25Q_CS_HIGH();

            return w25q_wait_busy();
        }

        case W25Q_CHIP_ERASE:
            w25q_write_enable();
            W25Q_CS_LOW();
            spi_transfer_byte(CMD_CHIP_ERASE);
            W25Q_CS_HIGH();
            return w25q_wait_busy();

        default:
            return -ENOTSUP;
    }
}

// ----------------------------------------------------------------------------
// Public device instance
// ----------------------------------------------------------------------------
static const drv_face_t dev_w25q = {
    .read = w25q_read,
    .write = w25q_write,
    .ioctl = w25q_ioctl
};

const drv_face_t* dev_w25q_get(void) {
    return &dev_w25q;
}