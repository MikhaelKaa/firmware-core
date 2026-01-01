/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#include "drv_face.h"

#define DRV_NUM (4U)

static drv_face_t * drv_table[DRV_NUM] = {0};

ssize_t drv_table_set(const drv_face_t* dev, unsigned int pos) {

    if(pos >= DRV_NUM) return -EINVAL;

    drv_table[pos] = (drv_face_t*)dev; 
    
    return 0;
}

ssize_t drv_table_get(drv_face_t** dev, unsigned int pos) {

    if(pos >= DRV_NUM) return -EINVAL;

    *dev = drv_table[pos]; 
    
    return 0;
}