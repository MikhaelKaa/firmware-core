/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

/**
 * @file stubs.c
 * @brief Реализация стубов для хост-тестов firmware-core.
 *
 * Определяет моковые экземпляры периферии (DWT, CoreDebug) и функции
 * управления мок-счётчиком циклов.
 */

#include <stdio.h>
#include <stdlib.h>
#include "cmsis_stubs.h"
#include "stubs.h"

/* ------------------------------------------------------------------ */
/* Моковые экземпляры периферии                                        */
/* ------------------------------------------------------------------ */

volatile uint32_t mock_dwt_ctrl  = 0;
volatile uint32_t mock_coredebug_demcr = 0;

DWT_Type       mock_dwt       = {0, 0};
CoreDebug_Type mock_coredebug = {0};

uint32_t g_mock_cyccnt     = 0;
uint32_t system_core_clock = 168000000U;  /* 168 МГц — как на целевой плате */

/* ------------------------------------------------------------------ */
/* Инициализация / управление CYCCNT                                   */
/* ------------------------------------------------------------------ */

void stubs_init(void)
{
    g_mock_cyccnt       = 0;
    system_core_clock   = 168000000U;
    mock_dwt.CTRL       = 0;
    mock_dwt.CYCCNT     = 0;
    mock_coredebug.DEMCR = 0;
}

void stubs_reset_cycles(void)
{
    g_mock_cyccnt   = 0;
    mock_dwt.CYCCNT = 0;
}

void stubs_advance_cycles(uint32_t cycles)
{
    g_mock_cyccnt   += cycles;
    mock_dwt.CYCCNT  = g_mock_cyccnt;
}

void stubs_advance_time_us(uint32_t us)
{
    uint32_t cycles = (uint32_t)((uint64_t)us * (system_core_clock / 1000000U));
    stubs_advance_cycles(cycles);
}

/* ------------------------------------------------------------------ */
/* hardware_panic() стуб                                               */
/* ------------------------------------------------------------------ */

void hardware_panic(const char *file, int line, const char *reason)
{
    fprintf(stderr, "PANIC at %s:%d - %s\n", file, line, reason);
    abort();
}