/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */


#include <string.h>
#include <errno.h>
#include "mem.h"
#include "stm32f4xx.h"

#define FLASH_BASE_ADDR          (0x08000000U)
#define FLASH_KEY1               (0x45670123U)
#define FLASH_KEY2               (0xCDEF89ABU)

// STM32F407VGT6 memory regions (из линкерского скрипта)
static const mem_region_t memory_regions[] = {
    {0x20000000U, 128U  * 1024U,     "RAM   "},    // Основная RAM
    {0x10000000U, 64U   * 1024U,     "CCMRAM"},    // CCM RAM
    {FLASH_BASE_ADDR, 1024U * 1024U, "FLASH "},    // FLASH 1024KB
};

static const dev_memory_t memory_info = {
    .num_of_regions = (sizeof(memory_regions) / sizeof(memory_regions[0])),
    .regions        = (mem_region_t*)memory_regions
};

// Current memory access address
static uintptr_t current_address = 0;

// Forward declarations
static int memory_read(void* buf, size_t len);
static int memory_write(const void* buf, size_t len);
static int memory_ioctrl(int cmd, void* arg);
static int memory_set_address(uintptr_t address);
static int memory_get_info(dev_memory_t** info);
static int memory_flash_unlock(void);
static int memory_flash_lock(void);
static int memory_flash_erase_sector(uint32_t sector);
static int memory_flash_write_word(uint32_t address, uint32_t data);
static int memory_flash_program(uint32_t address, const void* data, size_t len);
static uint32_t get_flash_sector(uint32_t address);
static uint32_t get_flash_sector_start(uint32_t sector);
static uint32_t get_flash_sector_size(uint32_t sector);

// Read from memory 
static int memory_read(void* buf, size_t len) {
    if (buf == NULL || len == 0) {
        return -EINVAL;
    }
    
    // Check if address is in valid region
    for (uint32_t i = 0; i < memory_info.num_of_regions; i++) {
        const mem_region_t* region = &memory_regions[i];
        if (current_address >= region->start && 
            (region->len == 0 || current_address < region->start + region->len)) {
            
            // For Flash and RAM regions - read directly
            const uint8_t* src = (const uint8_t*)current_address;
            memcpy(buf, src, len);
            current_address += len;
            return (int)len;
        }
    }
    
    return -EINVAL; // Address out of valid range
}

// Write to memory (interface implementation)
static int memory_write(const void* buf, size_t len) {
    if (buf == NULL || len == 0) {
        return -EINVAL;
    }
    
    // Check if address is in flash region
    for (uint32_t i = 0; i < memory_info.num_of_regions; i++) {
        const mem_region_t* region = &memory_regions[i];
        if (current_address >= region->start && 
            (region->len == 0 || current_address < region->start + region->len)) {
            
            // Check if we're trying to write to flash
            if (region->start == FLASH_BASE_ADDR) {
                return memory_flash_program(current_address, buf, len);
            }
            
            // For RAM regions
            uint8_t* dst = (uint8_t*)current_address;
            memcpy(dst, buf, len);
            current_address += len;
            return (int)len;
        }
    }
    
    return -EINVAL; // Address out of valid range
}

// Set current memory address
static int memory_set_address(uintptr_t address) {
    // Validate address is within known memory regions
    for (uint32_t i = 0; i < memory_info.num_of_regions; i++) {
        const mem_region_t* region = &memory_regions[i];
        if (address >= region->start && (region->len == 0 || address < region->start + region->len)) {
            current_address = address;
            return 0;
        }
    }
    return -EINVAL;
}

// Get memory device info
static int memory_get_info(dev_memory_t** info) {
    if (info == NULL) {
        return -EINVAL;
    }
    *info = (dev_memory_t*)&memory_info;
    return 0;
}

// Unlock Flash for writing
static int memory_flash_unlock(void) {
    // Check if Flash is already unlocked
    if (!(FLASH->CR & FLASH_CR_LOCK)) {
        return 0; // Already unlocked
    }
    
    // Perform unlock sequence
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
    
    // Verify unlock was successful
    if (FLASH->CR & FLASH_CR_LOCK) {
        return -EACCES; // Unlock failed
    }
    
    return 0;
}

// Lock Flash
static int memory_flash_lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;
    return 0;
}

// Get Flash sector from address (математическая формула)
static uint32_t get_flash_sector(uint32_t address) {
    uint32_t offset = address - FLASH_BASE_ADDR;
    
    // Сектора 0-3: по 16KB (0x4000)
    if (offset < 0x10000) { // 4 * 16KB = 64KB
        return offset / 0x4000;  // 16KB сектора
    }
    // Сектор 4: 64KB (0x10000)
    else if (offset < 0x20000) { // 64KB + 64KB = 128KB
        return 4;
    }
    // Сектора 5-11: по 128KB (0x20000)
    else {
        uint32_t big_offset = offset - 0x20000;
        if (big_offset < (7 * 0x20000)) { // 7 * 128KB = 896KB
            return 5 + (big_offset / 0x20000);
        }
    }
    
    return 0xFF; // Invalid
}

// Get Flash sector start address
static uint32_t get_flash_sector_start(uint32_t sector) {
    if (sector < 4) {
        return FLASH_BASE_ADDR + (sector * 0x4000); // 16KB
    } else if (sector == 4) {
        return FLASH_BASE_ADDR + 0x10000; // 64KB
    } else if (sector <= 11) {
        return FLASH_BASE_ADDR + 0x20000 + ((sector - 5) * 0x20000); // 128KB
    }
    return 0;
}

// Get Flash sector size
static uint32_t get_flash_sector_size(uint32_t sector) {
    if (sector < 4) {
        return 16 * 1024;  // 16KB sectors
    } else if (sector == 4) {
        return 64 * 1024;  // 64KB sector
    } else if (sector <= 11) {
        return 128 * 1024; // 128KB sectors
    }
    return 0;
}

// Erase Flash sector
static int memory_flash_erase_sector(uint32_t sector) {
    // Wait for any ongoing operation
    while (FLASH->SR & FLASH_SR_BSY) {}
    
    // Check if sector is valid (0-11 for STM32F407)
    if (sector > 11) {
        return -EINVAL;
    }
    
    // Set parallelism to x32
    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= FLASH_CR_PSIZE_1; // 32-bit programming
    
    // Start erase operation
    FLASH->CR &= ~FLASH_CR_SNB;
    FLASH->CR |= (sector << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;
    
    // Wait for completion
    while (FLASH->SR & FLASH_SR_BSY) {}
    
    // Check for errors - используем правильные имена для STM32F4
    if (FLASH->SR & (FLASH_SR_PGAERR | FLASH_SR_WRPERR)) {
        FLASH->SR |= (FLASH_SR_PGAERR | FLASH_SR_WRPERR);
        return -EIO;
    }
    
    // Clear end of operation flag
    if (FLASH->SR & FLASH_SR_EOP) {
        FLASH->SR |= FLASH_SR_EOP;
    }
    
    return 0;
}

// Write Flash word (32-bit programming для STM32F4)
static int memory_flash_write_word(uint32_t address, uint32_t data) {
    // Check address alignment (must be 4-byte aligned)
    if ((address & 0x3) != 0) {
        return -EINVAL;
    }
    
    // Wait for any ongoing operation
    while (FLASH->SR & FLASH_SR_BSY) {}
    
    // Set programming mode with x32 parallelism
    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= FLASH_CR_PSIZE_1; // 32-bit programming
    FLASH->CR |= FLASH_CR_PG;
    
    // Program word
    *(__IO uint32_t*)address = data;
    
    // Wait for completion
    while (FLASH->SR & FLASH_SR_BSY) {}
    
    // Check for errors - используем правильные имена для STM32F4
    if (FLASH->SR & (FLASH_SR_PGAERR | FLASH_SR_WRPERR)) {
        FLASH->CR &= ~FLASH_CR_PG;
        FLASH->SR |= (FLASH_SR_PGAERR | FLASH_SR_WRPERR);
        return -EIO;
    }
    
    // Clear programming flag
    FLASH->CR &= ~FLASH_CR_PG;
    
    return 0;
}

// Проверка, что область стерта (все байты = 0xFF)
static int is_flash_area_erased(uint32_t address, size_t len) {
    const uint8_t* ptr = (const uint8_t*)address;
    
    for (size_t i = 0; i < len; i++) {
        if (ptr[i] != 0xFF) {
            return 0; // Не стерто
        }
    }
    
    return 1; // Стерто
}

// High-level Flash programming без буфера
static int memory_flash_program(uint32_t address, const void* data, size_t len) {
    if (address < FLASH_BASE_ADDR || data == NULL || len == 0) {
        return -EINVAL;
    }
    
    // Check address alignment for Flash operations
    if ((address & 0x3) != 0) {
        return -EINVAL; // Адрес должен быть выровнен по 4 байта
    }
    
    // Длина должна быть кратна 4 для 32-битной записи
    if ((len & 0x3) != 0) {
        return -EINVAL;
    }
    
    // Определяем сектор
    uint32_t sector = get_flash_sector(address);
    if (sector == 0xFF) {
        return -EINVAL;
    }
    
    // Определяем границы сектора
    uint32_t sector_start = get_flash_sector_start(sector);
    uint32_t sector_size = get_flash_sector_size(sector);
    uint32_t sector_end = sector_start + sector_size;
    
    // Проверяем, что запись не выходит за границы сектора
    if ((address + len) > sector_end) {
        return -EINVAL; // Запись выходит за границы сектора
    }
    
    // Проверяем, что область стерта
    if (!is_flash_area_erased(address, len)) {
        return -EACCES; // Область не стерта, нужна операция стирания сектора
    }
    
    // Unlock Flash
    int result = memory_flash_unlock();
    if (result != 0) {
        return result;
    }
    
    // Пишем данные по 32-битным словам
    const uint32_t* src = (const uint32_t*)data;
    uint32_t* dst = (uint32_t*)address;
    
    for (size_t i = 0; i < len / 4; i++) {
        // Проверяем, что пытаемся записать не 0xFFFFFFFF (это уже стертое состояние)
        if (src[i] != 0xFFFFFFFF) {
            result = memory_flash_write_word((uint32_t)&dst[i], src[i]);
            if (result != 0) {
                memory_flash_lock();
                return result;
            }
        }
    }
    
    // Lock Flash
    memory_flash_lock();
    
    current_address += len;
    return (int)len;
}

// IO Control for memory (interface implementation)
static int memory_ioctrl(int cmd, void* arg) {
    switch (cmd) {
        case MEMORY_SET_ADDRESS:
            if (arg == NULL) return -EINVAL;
            return memory_set_address(*(uintptr_t*)arg);
            
        case MEMORY_GET_INFO:
            return memory_get_info((dev_memory_t**)arg);

        case MEMORY_GET_ADDRESS:
            if (arg == NULL) return -EINVAL;
            *(uintptr_t*)arg = current_address;
            return 0;

        case MEMORY_FLASH_UNLOCK:
            return memory_flash_unlock();
            
        case MEMORY_FLASH_LOCK:
            return memory_flash_lock();
            
        case MEMORY_FLASH_ERASE_SECTOR:
            if (arg == NULL) return -EINVAL;
            return memory_flash_erase_sector(*(uint32_t*)arg);
            
        case MEMORY_FLASH_ERASE_MASS:
            // Mass erase not implemented for safety
            return -ENOTSUP;
            
        default:
            return -ENOTSUP;
    }
}

// Memory device instance (static - hidden inside module)
static const drv_face_t dev_memory = {
    .read = memory_read,
    .write = memory_write,
    .ioctl = memory_ioctrl
};

// Memory device instance accessor
const drv_face_t* dev_memory_get(void) {
    return &dev_memory;
}