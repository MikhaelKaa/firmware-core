/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

/**
 * @file cmsis_stubs.h
 * @brief Моки CMSIS-регистров для хост-тестов (x86_64).
 *
 * Заменяет реальные структуры DWT, CoreDebug и константы масок,
 * чтобы precise_time.h компилировался без зависимости от STM32.
 *
 * Механизм: до подключения CMSIS-заголовка определяем его guard-макрос
 * (__STM32F407xx_H), поэтому #include "stm32f407xx.h" в precise_time.h
 * становится no-op. Все необходимые типы и маски объявлены здесь.
 *
 * DWT и CoreDebug указывают на глобальные переменные mock_dwt /
 * mock_coredebug, поля которых привязаны к g_mock_cyccnt через макросы
 * в stubs.c (не требуется — достаточно прямого доступа).
 *
 * ВАЖНО: этот заголовок должен быть включён ДО precise_time.h, либо
 * точка include search path test_helpers/ должна идти раньше путей к CMSIS.
 */

#ifndef CMSIS_STUBS_H
#define CMSIS_STUBS_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Предотвращаем подключение настоящего stm32f407xx.h                   */
/* ------------------------------------------------------------------ */

#define __STM32F407xx_H  1

/* ------------------------------------------------------------------ */
/* Глобальные моки                                                     */
/* ------------------------------------------------------------------ */

/** Виртуальный счётчик циклов (привязан к mock_dwt.CYCCNT).        */
extern uint32_t g_mock_cyccnt;

/** Моковый системный тактовый генератор (по умолчанию 168 МГц).    */
extern uint32_t system_core_clock;

/* ------------------------------------------------------------------ */
/* Маски (как в CMSIS для Cortex-M4)                                   */
/* ------------------------------------------------------------------ */

#define CoreDebug_DEMCR_TRCENA_Msk  0x01000000U
#define DWT_CTRL_CYCCNTENA_Msk      0x00000001U

/* ------------------------------------------------------------------ */
/* Моковые структуры периферии                                         */
/* ------------------------------------------------------------------ */

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
} DWT_Type;

typedef struct {
    volatile uint32_t DEMCR;
} CoreDebug_Type;

/* Глобальные экземпляры */
extern DWT_Type       mock_dwt;
extern CoreDebug_Type mock_coredebug;

#define  DWT        (&mock_dwt)
#define CoreDebug   (&mock_coredebug)

#endif /* CMSIS_STUBS_H */