/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#ifndef _SVCCALLS_H
#define _SVCCALLS_H

#include "drv_face.h"
#include <stdint.h>

#define SVC_DRV_TABLE_SET (0x0f + 0x00)
#define SVC_DRV_TABLE_GET (0x0f + 0x01)

// Вспомогательная функция для SVC вызова с возвращаемым значением
inline ssize_t svc_call_return(uint32_t num, uint32_t arg0, uint32_t arg1, uint32_t arg2, uint32_t arg3)
{
    register uint32_t result asm("r0");
    __asm volatile(
        "mov r0, %1\n"
        "mov r1, %2\n"
        "mov r2, %3\n"
        "mov r3, %4\n"
        "svc %5\n"
        : "=r" (result)
        : "r" (arg0), "r" (arg1), "r" (arg2), "r" (arg3), "I" (num)
        : "r1", "r2", "r3", "memory"
    );
    return (ssize_t)result;
}

inline ssize_t fc_drv_table_set(const drv_face_t* dev, signed int pos)
{
    return svc_call_return(SVC_DRV_TABLE_SET, (uint32_t)dev, (uint32_t)pos, 0, 0);
}

inline ssize_t fc_drv_table_get(drv_face_t** dev, signed int pos)
{
    return svc_call_return(SVC_DRV_TABLE_GET, (uint32_t)dev, (uint32_t)pos, 0, 0);
}

#endif /* _SVCCALLS_H */