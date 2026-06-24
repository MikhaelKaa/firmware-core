/**
 * @file micros.h
 *
 * @deprecated ЭТОТ ФАЙЛ УСТАРЕЛ. Используйте precise_time.h вместо него.
 *
 * Причина устаревания
 * -------------------
 * Функции micros() и millis() возвращают абсолютное время в микросекундах/миллисекундах,
 * но их диапазон сжат (0 → ~25 565 281 при 168 МГц из-за переполнения DWT->CYCCNT).
 *
 * При попытке вычислить разницу между двумя вызовами через границу переполнения:
 *
 *     uint32_t start = micros();  // ~25 000 000
 *     uint32_t end   = micros();  // ~100 000 (после переполнения)
 *     uint32_t diff  = end - start;  // ≈ 4 269 967 296 — ЗАВЕДОМО НЕВЕРНО!
 *
 * Беззнаковая арифметика ломается, потому что значения не покрывают полный диапазон uint32_t.
 *
 * Миграция (старое -> новое)
 * -------------------------
 *     us_init()          → pt_init()
 *     delay_us(us)       → pt_delay_us(us)
 *     micros()           → pt_now_us()   (только для логирования!)
 *     millis()           → pt_now_ms()   (только для логирования!)
 *
 * Для корректного вычисления интервалов:
 *     uint32_t t0 = pt_stamp();
 *     // ... операция ...
 *     uint32_t elapsed = pt_elapsed_us(t0);
 *
 * @see precise_time.h — полный API, документация и примеры.
 */

#ifndef DWT_MICROS
#define DWT_MICROS

#include "precise_time.h"

/* Обратная совместимость: старые имена -> новый API                */

#define us_init()          pt_init()
#define delay_us(us)       pt_delay_us(us)
#define micros()           pt_now_us()
#define millis()           pt_now_ms()

#endif /* DWT_MICROS */
