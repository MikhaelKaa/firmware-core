/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

#ifndef _RESET_H_
#define _RESET_H_

/**
 * @brief Выполнить сброс MCU (не возвращается).
 *
 * Платформозависимая функция — использует CMSIS NVIC_SystemReset().
 */
void reset_perform(void);

#endif /* _RESET_H_ */