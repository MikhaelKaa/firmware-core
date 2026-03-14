/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#ifndef DEV_RTC_H
#define DEV_RTC_H

#include "drv_face.h"
#include <stdint.h>

#define RTC_MAGIC_NUMBER    (0xF55FA00AU)

void RTC_init(void);

#endif /* DEV_RTC_H */