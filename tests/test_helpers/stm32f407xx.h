/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

/**
 * @file stm32f407xx.h
 * @brief Пустая заглушка CMSIS-заголовка для хост-тестов.
 *
 * Перехватывает #include "stm32f407xx.h" из precise_time.h,
 * чтобы не тащить 15k строк реальной CMSIS при компиляции на x86_64.
 * Все необходимые типы и маски уже определены в cmsis_stubs.h.
 */

/* Guard уже задан в cmsis_stubs.h — дублируем для самодостаточности */
#ifndef __STM32F407xx_H
#define __STM32F407xx_H

/* Пусто: типы DWT_Type, CoreDebug_Type и макросы DWT/CoreDebug
 * объявлены в cmsis_stubs.h, который подключается ДО этого файла. */

#endif /* __STM32F407xx_H */