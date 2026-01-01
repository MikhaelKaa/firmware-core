/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#ifndef _SVCCALLS_H
#define _SVCCALLS_H

#include "drv_face.h"

#define SVC_DRV_TABLE_SET (0x0f + 0x00)
#define SVC_DRV_TABLE_GET (0x0f + 0x01)

// Макрос для вызова SVC с номером вызова
#define SVC_CALL(num, arg0, arg1, arg2, arg3) \
    __asm volatile(                           \
        "mov r0, %0\n"                        \
        "mov r1, %1\n"                        \
        "mov r2, %2\n"                        \
        "mov r3, %3\n"                        \
        "svc %4\n"                            \
        :                                     \
        : "r" (arg0), "r" (arg1), "r" (arg2), "r" (arg3), "I" (num) \
        : "r0", "r1", "r2", "r3", "memory"    \
    )


#define SVC_CALL_RETURN(num, arg0, arg1, arg2, arg3) \
({ \
    register uint32_t result asm ("r0"); \
    __asm volatile( \
        "mov r0, %1\n" \
        "mov r1, %2\n" \
        "mov r2, %3\n" \
        "mov r3, %4\n" \
        "svc %5\n" \
        : "=r" (result) \
        : "r" (arg0), "r" (arg1), "r" (arg2), "r" (arg3), "I" (num) \
        : "r1", "r2", "r3", "memory" \
    ); \
    (ssize_t)result; \
})

inline ssize_t fc_drv_table_set(const drv_face_t* dev, signed int pos){
    return SVC_CALL_RETURN(SVC_DRV_TABLE_SET, dev, pos, 0, 0);
}

inline ssize_t fc_drv_table_get(drv_face_t** dev, signed int pos) {
    return SVC_CALL_RETURN(SVC_DRV_TABLE_GET, dev, pos, 0, 0);
}

#endif /* _SVCCALLS_H */