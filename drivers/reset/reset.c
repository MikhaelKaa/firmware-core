/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

#include "stm32f407xx.h"
#include "reset.h"

/**
 * @brief Выполнить сброс MCU (не возвращается).
 *
 * Использует CMSIS NVIC_SystemReset() для аппаратного сброса.
 */
void reset_perform(void)
{
    NVIC_SystemReset();
}