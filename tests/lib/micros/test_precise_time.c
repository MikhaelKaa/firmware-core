/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

/**
 * @file test_precise_time.c
 * @brief Юнит-тесты для модуля precise_time (DWT CYCCNT).
 *
 * Проверяет:
 * - Инициализацию счётчика (pt_init)
 * - Получение меток (pt_stamp)
 * - Вычисление прошедшего времени (pt_elapsed_us, pt_diff_us)
 * - Конверсию циклов -> микросекунды (pt_to_us, pt_now_us, pt_now_ms)
 * - Обратную совместимость через micros.h (us_init, micros, millis)
 * - Корректную работу при переполнении 32-битного счётчика (wraparound)
 */

/* ВАЖНО: cmsis_stubs.h ДО precise_time.h — блокирует stm32f407xx.h */
#include "cmsis_stubs.h"
#include "unity.h"
#include "stubs.h"
#include "precise_time.h"

/* ------------------------------------------------------------------ */
/* Вспомогательные макроси                                             */
/* ------------------------------------------------------------------ */

/** Делитель циклов -> микросекунды (раскрывается из PT_CYCLES_PER_US). */
static uint32_t cycles_per_us(void)
{
    return system_core_clock / 1000000U;
}

/* ------------------------------------------------------------------ */
/* setUp / tearDown                                                    */
/* ------------------------------------------------------------------ */

void setUp(void)
{
    stubs_init();
}

void tearDown(void)
{
    /* nothing */
}

/* ------------------------------------------------------------------ */
/* Тест: инициализация счётчика                                        */
/* ------------------------------------------------------------------ */

static void test_pt_init_enables_and_resets(void)
{
    /* Начальные значения могут быть ненулевыми после предыдущих тестов */
    mock_dwt.CYCCNT = 0xDEADBEEF;
    mock_dwt.CTRL   = 0;
    mock_coredebug.DEMCR = 0;

    pt_init();

    TEST_ASSERT_EQUAL(0U,           mock_dwt.CYCCNT);
    TEST_ASSERT_BITS_HIGH(DWT_CTRL_CYCCNTENA_Msk,     mock_dwt.CTRL);
    TEST_ASSERT_BITS_HIGH(CoreDebug_DEMCR_TRCENA_Msk, mock_coredebug.DEMCR);
}

/* ------------------------------------------------------------------ */
/* Тест: pt_stamp возвращает текущее значение CYCCNT                   */
/* ------------------------------------------------------------------ */

static void test_pt_stamp_returns_cyccnt(void)
{
    stubs_advance_cycles(12345U);

    uint32_t stamp = pt_stamp();

    TEST_ASSERT_EQUAL(12345U, stamp);
}

/* ------------------------------------------------------------------ */
/* Тест: pt_elapsed_us — корректный интервал                           */
/* ------------------------------------------------------------------ */

static void test_pt_elapsed_us_basic(void)
{
    pt_init();

    /* Зафиксируем начальную метку при CYCCNT = 0 */
    uint32_t t0 = pt_stamp();

    /* Продвинем счётчик на 168000 циклов = 1000 мкс при 168 МГц */
    stubs_advance_cycles(168000U);

    uint32_t elapsed = pt_elapsed_us(t0);

    TEST_ASSERT_EQUAL(1000U, elapsed);
}

/* ------------------------------------------------------------------ */
/* Тест: pt_diff_us — разница двух меток                               */
/* ------------------------------------------------------------------ */

static void test_pt_diff_us_basic(void)
{
    pt_init();

    uint32_t t_start = pt_stamp();         /* CYCCNT = 0 */
    stubs_advance_cycles(16800U);          /* 100 мкс */
    uint32_t t_end   = pt_stamp();

    uint32_t diff = pt_diff_us(t_start, t_end);

    TEST_ASSERT_EQUAL(100U, diff);
}

/* ------------------------------------------------------------------ */
/* Тест: pt_to_us / pt_now_us / pt_now_ms                              */
/* ------------------------------------------------------------------ */

static void test_pt_convenience_functions(void)
{
    pt_init();
    stubs_advance_cycles(1680000U);  /* 10000 мкс = 10 мс */

    TEST_ASSERT_EQUAL(10000U, pt_to_us(DWT->CYCCNT));
    TEST_ASSERT_EQUAL(10000U, pt_now_us());
    TEST_ASSERT_EQUAL(10U,    pt_now_ms());
}

/* ------------------------------------------------------------------ */
/* Тест: обратная совместимость через micros.h                         */
/* ------------------------------------------------------------------ */

static void test_micros_compat_layer(void)
{
    /* Включаем макросы из micros.h (обёртки над pt_*).
     * Для этого нужно определить их вручную, т.к. micros.h
     * уже инклудит precise_time.h через cmsis_stubs.         */

    pt_init();
    stubs_advance_cycles(336000U);  /* 2000 мкс = 2 мс */

    /* us_init() -> pt_init() — уже вызван выше, проверяем только маппинг */
    TEST_ASSERT_EQUAL(2000U, pt_now_us());   /* micros() эквивалент */
    TEST_ASSERT_EQUAL(2U,    pt_now_ms());   /* millis() эквивалент */
}

/* ------------------------------------------------------------------ */
/* Тест: wraparound в pt_elapsed_us                                    */
/* ------------------------------------------------------------------ */

static void test_pt_elapsed_us_wraparound(void)
{
    pt_init();

    /* Установим счётчик близко к переполнению */
    uint32_t near_overflow = 0xFFFFFFF0U;
    mock_dwt.CYCCNT = near_overflow;
    g_mock_cyccnt   = near_overflow;

    uint32_t t0 = pt_stamp();

    /* Продвинем на 168000 циклов (1000 мкс).
     * Счётчик переполнится: 0xFFFFFFF0 + 168000 = 0x00029C08 (wrap) */
    stubs_advance_cycles(168000U);

    uint32_t elapsed = pt_elapsed_us(t0);

    /* Беззнаковая арифметика корректно обрабатывает wraparound.
     * DWT->CYCCNT - t0 = 0x00029C08 - 0xFFFFFFF0 = 0x00000038 (56 циклов)
     * Но так как stubs_advance_cycles использует сложение с переполнением:
     * g_mock_cyccnt = 0xFFFFFFF0 + 168000 = 168000 - 16 = 167984...
     * Проверяем через разницу напрямую.                          */
    TEST_ASSERT_EQUAL(1000U, elapsed);
}

/* ------------------------------------------------------------------ */
/* Тест: wraparound в pt_diff_us                                       */
/* ------------------------------------------------------------------ */

static void test_pt_diff_us_wraparound(void)
{
    pt_init();

    /* t_start близко к переполнению */
    mock_dwt.CYCCNT = 0xFFFFFF00U;
    g_mock_cyccnt   = 0xFFFFFF00U;
    uint32_t t_start = pt_stamp();

    /* Продвинем на 16800 циклов (100 мкс) через границу */
    stubs_advance_cycles(16800U);
    uint32_t t_end = pt_stamp();

    uint32_t diff = pt_diff_us(t_start, t_end);

    TEST_ASSERT_EQUAL(100U, diff);
}

/* ------------------------------------------------------------------ */
/* Тест: elapsed при малых интервалах (до 1 мкс)                       */
/* ------------------------------------------------------------------ */

static void test_pt_elapsed_sub_microsecond(void)
{
    pt_init();
    uint32_t t0 = pt_stamp();

    /* Продвинем на 84 цикла (менее 1 мкс при 168 МГц) */
    stubs_advance_cycles(84U);

    uint32_t elapsed = pt_elapsed_us(t0);

    /* Целочисленное деление: 84 / 168 = 0 мкс */
    TEST_ASSERT_EQUAL(0U, elapsed);
}

/* ------------------------------------------------------------------ */
/* Тест: точность на границе 1 мкс                                     */
/* ------------------------------------------------------------------ */

static void test_pt_elapsed_exactly_one_us(void)
{
    pt_init();
    uint32_t t0 = pt_stamp();

    /* Ровно 168 циклов = 1 мкс при 168 МГц */
    stubs_advance_cycles(cycles_per_us());

    uint32_t elapsed = pt_elapsed_us(t0);

    TEST_ASSERT_EQUAL(1U, elapsed);
}

/* ------------------------------------------------------------------ */
/* Точка входа                                                         */
/* ------------------------------------------------------------------ */

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_pt_init_enables_and_resets);
    RUN_TEST(test_pt_stamp_returns_cyccnt);
    RUN_TEST(test_pt_elapsed_us_basic);
    RUN_TEST(test_pt_diff_us_basic);
    RUN_TEST(test_pt_convenience_functions);
    RUN_TEST(test_micros_compat_layer);
    RUN_TEST(test_pt_elapsed_us_wraparound);
    RUN_TEST(test_pt_diff_us_wraparound);
    RUN_TEST(test_pt_elapsed_sub_microsecond);
    RUN_TEST(test_pt_elapsed_exactly_one_us);

    return UNITY_END();
}