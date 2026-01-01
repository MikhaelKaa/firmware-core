/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include "svccalls.h"
#include "drv_face.h"


__attribute__((naked)) void SVC_Handler(void)
{
    __asm volatile(
        // MSP or PSP?
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r1, msp\n"    // MSP
        "mrsne r1, psp\n"    // PSP

        "push {r1, lr}\n"

        // Получаем PC из стека
        "ldr r2, [r1, #24]\n"

        // Получаем адрес инструкции SVC
        "subs r2, #2\n"         // PC указывает на следующую инструкцию после SVC
        
        // Читаем инструкцию SVC (16 бит в Thumb)
        "ldrh r0, [r2]\n"
        
        // Извлекаем номер SVC 
        "and r0, #0xff\n"
        
        // <---
        "bl svc_proc\n"

        "pop {r1, lr}\n"
        
        "str r0, [r1, #0]\n"
        
        "bx lr\n"
    );
}

uint32_t svc_proc(uint32_t svc, uint32_t* arg) {

    switch (svc)
    {
    case SVC_DRV_TABLE_SET:
        return (uint32_t) drv_table_set((const drv_face_t*)arg[0], arg[1]);
        break;

    case SVC_DRV_TABLE_GET:
        return (uint32_t) drv_table_get((drv_face_t**)arg[0], arg[1]);
        break;
    
    default:
        return (uint32_t)-EINVAL;
        break;
    }
}