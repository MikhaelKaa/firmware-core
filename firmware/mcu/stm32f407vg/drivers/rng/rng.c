/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#include "rng.h"
#include "stm32f407xx.h"
#include <string.h>
#include <errno.h>

// Internal state
static volatile uint32_t rng_status = 0;

// Open RNG (interface implementation)
static int rng_init(void) {
    // Enable RNG clock
    RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
    
    // Enable the RNG
    RNG->CR |= RNG_CR_RNGEN;
    
    // Clear any pending errors
    RNG->SR = 0;
    
    rng_status = RNG_STATUS_READY;
    return 0;
}

// Close RNG (interface implementation)
static int rng_deinit(void) {
    // Disable the RNG
    RNG->CR &= ~RNG_CR_RNGEN;
    
    // Disable RNG clock
    RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN;
    
    rng_status = 0;
    return 0;
}

// Read random data from RNG (interface implementation)
static int rng_read(void *buf, size_t count) {
    if (buf == NULL || count == 0) {
        return -EINVAL;
    }
    
    uint8_t *buffer = (uint8_t *)buf;
    size_t bytes_read = 0;
    uint32_t timeout;
    
    while (bytes_read < count) {
        // Check if data is ready
        timeout = RNG_TIMEOUT;
        while (!(RNG->SR & RNG_SR_DRDY) && timeout--) {
            if (RNG->SR & (RNG_SR_SECS | RNG_SR_CECS)) {
                rng_status |= RNG_STATUS_ERROR;
                if (RNG->SR & RNG_SR_SECS) rng_status |= RNG_STATUS_SEED_ERROR;
                if (RNG->SR & RNG_SR_CECS) rng_status |= RNG_STATUS_CLOCK_ERROR;
                return -EIO;
            }
        }
        
        if (timeout == 0) {
            return -ETIMEDOUT;
        }
        
        // Read random data
        uint32_t random_data = RNG->DR;
        
        // Copy to buffer
        size_t bytes_to_copy = count - bytes_read;
        if (bytes_to_copy > 4) bytes_to_copy = 4;
        
        memcpy(&buffer[bytes_read], &random_data, bytes_to_copy);
        bytes_read += bytes_to_copy;
    }
    
    return (int)bytes_read;
}

// Write to RNG (interface implementation) - not supported
static int rng_write(const void *buf, size_t count) {
    (void)(buf);
    (void)(count);
    // RNG is read-only device
    return -ENOTSUP;
}

// Get RNG status
static uint32_t rng_get_status(void) {
    rng_status = RNG_STATUS_READY;
    
    if (RNG->SR & RNG_SR_DRDY) {
        rng_status |= RNG_STATUS_READY;
    }
    if (RNG->SR & (RNG_SR_SECS | RNG_SR_CECS)) {
        rng_status |= RNG_STATUS_ERROR;
        if (RNG->SR & RNG_SR_SECS) rng_status |= RNG_STATUS_SEED_ERROR;
        if (RNG->SR & RNG_SR_CECS) rng_status |= RNG_STATUS_CLOCK_ERROR;
    }
    
    return rng_status;
}

// Reset RNG
static int rng_reset(void) {
    // Disable and re-enable RNG to reset it
    RNG->CR &= ~RNG_CR_RNGEN;
    
    // Clear status register
    RNG->SR = 0;
    
    // Re-enable RNG
    RNG->CR |= RNG_CR_RNGEN;
    
    rng_status = RNG_STATUS_READY;
    return 0;
}

// Simple self-test
static int rng_self_test(void) {
    uint32_t test_data[4];
    int result;
    
    // Read 4 random numbers
    result = rng_read(test_data, sizeof(test_data));
    if (result != sizeof(test_data)) {
        return -EIO;
    }
    
    // Check that we got some non-zero data (not perfect but basic check)
    int zeros = 0;
    for (int i = 0; i < 4; i++) {
        if (test_data[i] == 0) zeros++;
    }
    
    // If all numbers are zero, something is wrong
    if (zeros == 4) {
        return -EIO;
    }
    
    return 0;
}

// IO Control for RNG (interface implementation)
static int rng_ioctrl(int cmd, void *arg) {
    switch (cmd) {
        case INTERFACE_INIT:
            return rng_init();
            
        case INTERFACE_DEINIT:
            return rng_deinit();

        case RNG_GET_STATUS:
            if (arg != NULL) {
                *(uint32_t *)arg = rng_get_status();
            }
            return 0;
            
        case RNG_RESET:
            return rng_reset();
            
        case RNG_SELF_TEST:
            return rng_self_test();
            
        default:
            return -ENOTSUP;
    }
}

// RNG device instance
static const interface_t dev_rng = {
    .read = rng_read,
    .write = rng_write,
    .ioctl = rng_ioctrl
};

// RNG device instance accessor
const interface_t* dev_rng_get(void) {
    return (const interface_t*)&dev_rng;
}