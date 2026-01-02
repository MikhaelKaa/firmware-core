/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#ifndef MEMORY_F407_H
#define MEMORY_F407_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include "drv_face.h"

/* Memory region descriptor */
typedef struct {
    uintptr_t  start;   /* Start address of memory region */
    size_t len;     /* Length of memory region in bytes */
    char name[10];
} mem_region_t;

/* Memory device descriptor */
typedef struct {
    uint32_t num_of_regions;    /* Number of memory regions */
    mem_region_t* regions;      /* Pointer to array of memory regions */
} dev_memory_t;

/* Memory-specific ioctl commands */
#define MEMORY_GET_INFO          (INTERFACE_CMD_DEVICE + 0)  // Get memory device info
#define MEMORY_SET_ADDRESS       (INTERFACE_CMD_DEVICE + 1)  // Set current memory address
#define MEMORY_GET_ADDRESS       (INTERFACE_CMD_DEVICE + 2)  // Get current memory address
#define MEMORY_FLASH_UNLOCK      (INTERFACE_CMD_DEVICE + 3)  // Unlock Flash for writing
#define MEMORY_FLASH_LOCK        (INTERFACE_CMD_DEVICE + 4)  // Lock Flash
#define MEMORY_FLASH_ERASE_SECTOR (INTERFACE_CMD_DEVICE + 5) // Erase Flash sector
#define MEMORY_FLASH_ERASE_MASS  (INTERFACE_CMD_DEVICE + 6)  // Mass erase Flash

/* Global memory device instance accessor */
const drv_face_t* dev_memory_get(void);

static inline void dev_memory_print_info(void) {
    const drv_face_t* mem_dev = dev_memory_get();
    if (mem_dev == NULL) {
        printf("Memory device not available\r\n");
        return;
    }
    
    dev_memory_t* mem_info = NULL;
    int result = mem_dev->ioctl(MEMORY_GET_INFO, &mem_info);
    if (result != 0 || mem_info == NULL) {
        printf("Failed to get memory info: %d\r\n", result);
        return;
    }
    
    printf("Memory: num of regions: %lu\r\n", mem_info->num_of_regions);
    for(size_t i = 0; i < mem_info->num_of_regions; i++) {
        printf("%u %s \tstart 0x%08" PRIxPTR "  size 0x%08x\r\n", 
               i, 
               mem_info->regions[i].name, 
               mem_info->regions[i].start, 
               mem_info->regions[i].len);
    }
}

#endif /* MEMORY_F407_H */