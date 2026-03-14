/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#ifndef DEV_RTC_H
#define DEV_RTC_H

#include "drv_face.h"
#include <stdint.h>

extern const uint32_t rtc_magic_number;

void RTC_init(void);

#endif /* DEV_RTC_H */