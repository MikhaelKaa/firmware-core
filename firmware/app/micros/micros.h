
#ifndef DWT_MICROS
#define DWT_MICROS

#include "stm32f407xx.h"

extern uint32_t system_core_clock;

static inline void us_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Разрешаем использовать счётчик.
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;     // Запускаем счётчик.
}

static inline  void delay_us(uint32_t us) {
    // На первый взгляд работает. Точных замеров не делал, 
    // наверное стоит добавить калибровочные значения.
    uint32_t us_count_tick = (uint32_t)((uint64_t)us * system_core_clock / 1000000U);
    uint32_t start_tick = DWT->CYCCNT;
    
    // Учитываем переполнение счётчика:
    while ((DWT->CYCCNT - start_tick) < us_count_tick);
}
 
static inline  uint32_t micros(void) {
    return  DWT->CYCCNT / (system_core_clock / 1000000U);
}

// TODO: test me
static inline  uint32_t millis(void) {
    return  DWT->CYCCNT / (system_core_clock / 1000000U) / 1000U;
}


#endif /* DWT_MICROS */
