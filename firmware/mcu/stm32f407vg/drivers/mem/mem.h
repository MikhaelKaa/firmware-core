/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#ifndef MEMORY_F407_H
#define MEMORY_F407_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include "drv_face.h"

// Описание региона памяти
typedef struct {
    uintptr_t  start;   /* Начальный адрес региона памяти */
    size_t len;     /* Длина региона памяти в байтах */
    char name[10];
} mem_region_t;

// Описание устройства памяти
typedef struct {
    uint32_t num_of_regions;    /* Количество регионов памяти */
    mem_region_t* regions;      /* Указатель на массив регионов памяти */
} dev_memory_t;

// Команды ioctl для управления памятью
#define MEMORY_GET_INFO          (INTERFACE_CMD_DEVICE + 0)  // Получение информации о устройстве памяти
#define MEMORY_SET_ADDRESS       (INTERFACE_CMD_DEVICE + 1)  // Установка текущего адреса в памяти
#define MEMORY_GET_ADDRESS       (INTERFACE_CMD_DEVICE + 2)  // Получение текущего адреса в памяти
#define MEMORY_FLASH_UNLOCK      (INTERFACE_CMD_DEVICE + 3)  // Разблокировка Flash для записи
#define MEMORY_FLASH_LOCK        (INTERFACE_CMD_DEVICE + 4)  // Блокировка Flash
#define MEMORY_FLASH_ERASE_SECTOR (INTERFACE_CMD_DEVICE + 5) // Стирание сектора Flash
#define MEMORY_FLASH_ERASE_MASS  (INTERFACE_CMD_DEVICE + 6) // Массовое стирание Flash

// Получение экземпляра устройства памяти
const drv_face_t* dev_memory_get(void);

// Вывод информации о памяти
static inline void dev_memory_print_info(void) {
    const drv_face_t* mem_dev = dev_memory_get();
    if (mem_dev == NULL) {
        printf("Устройство памяти недоступно\r\n");
        return;
    }
    
    dev_memory_t* mem_info = NULL;
    int result = mem_dev->ioctl(MEMORY_GET_INFO, &mem_info);
    if (result != 0 || mem_info == NULL) {
        printf("Не удалось получить информацию о памяти: %d\r\n", result);
        return;
    }
    
    printf("Память: количество регионов: %lu\r\n", mem_info->num_of_regions);
    for(size_t i = 0; i < mem_info->num_of_regions; i++) {
        printf("%u %s \tначало 0x%08" PRIxPTR "  размер 0x%08x\r\n", 
               i, 
               mem_info->regions[i].name, 
               mem_info->regions[i].start, 
               mem_info->regions[i].len);
    }
}

#endif /* MEMORY_F407_H */
