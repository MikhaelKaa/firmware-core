/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */
// System init for stm32f407 board
// 03.11.2025

#include "stm32f407xx.h"

void SystemInit(void)
{
    volatile uint32_t timeout = 0;

    // Включение тактирования Backup Domain и PWR
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    
    // Разрешаем доступ к Backup Domain (обязательно для LSE)
    PWR->CR |= PWR_CR_DBP;
    
    // Сначала инициализируем LSE до основной PLL настройки
    RCC->BDCR |= RCC_BDCR_LSEON;
    
    // Ждем готовности LSE
    timeout = 2000000; // Увеличим таймаут для LSE (он медленный)
    while(!(RCC->BDCR & RCC_BDCR_LSERDY) && timeout--) {
        __NOP();
    }
    
    // Если LSE не запустился, проверяем флаг ошибки
    if (RCC->BDCR & RCC_BDCR_LSERDY) {
        // LSE успешно запущен
        // Настраиваем RTC на использование LSE
        // Включаем RTC
        RCC->BDCR |= (1 << 8);
        RCC->BDCR |= RCC_BDCR_RTCEN;
    } else {
        // LSE не запустился - возможно аппаратная проблема
        // Здесь можно добавить обработку ошибки
        // Выключаем LSE
        RCC->BDCR &= ~RCC_BDCR_LSEON;
        
        // Альтернатива: используем LSI для RTC
        RCC->CSR |= RCC_CSR_LSION;
        timeout = 100000;
        while(!(RCC->CSR & RCC_CSR_LSIRDY) && timeout--) {
            __NOP();
        }
        // Включаем RTC
        RCC->BDCR |= (2 << 8);
        RCC->BDCR |= RCC_BDCR_RTCEN;
    }
    
    // Настройка регулятора напряжения
    // Voltage scaling range 1
    PWR->CR |= PWR_CR_VOS;
    
    // Включаем HSE
    RCC->CR |= RCC_CR_HSEON;
    // Ждем готовности HSE
    timeout = 1000000;
    while((!(RCC->CR & RCC_CR_HSERDY)) && timeout--) {
        __NOP();
    }
    // Настраиваем PLL
    RCC->PLLCFGR = (4 << 0)   |  // PLLM = 4
                   (168 << 6) |  // PLLN = 168  
                   (0 << 16)  |  // PLLP = 2
                   (7 << 24)  |  // PLLQ = 7
                   RCC_PLLCFGR_PLLSRC_HSE;
    // Variant pll
    // Настройка PLL: HSE -> PLL, 168 MHz
    // RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos) |    // PLLM = 8
    //                (336 << RCC_PLLCFGR_PLLN_Pos) |  // PLLN = 336
    //                (0 << RCC_PLLCFGR_PLLP_Pos) |    // PLLP = 2
    //                (7 << RCC_PLLCFGR_PLLQ_Pos) |    // PLLQ = 7
    //                RCC_PLLCFGR_PLLSRC_HSE;
    
    // Включаем PLL
    RCC->CR |= RCC_CR_PLLON;
    timeout = 1000000;
    // Ждем готовности PLL
    while(!(RCC->CR & RCC_CR_PLLRDY) && timeout--) {
        __NOP();
    }

    // Настройка делителей
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1   |  // AHB  = SYSCLK / 1 = 168MHz
                 RCC_CFGR_PPRE1_DIV4  |  // APB1 = AHB / 4 = 42MHz  
                 RCC_CFGR_PPRE2_DIV2;    // APB2 = AHB / 2 = 84MHz
    
    // Настройка Flash памяти для работы на 168 MHz
    FLASH->ACR = FLASH_ACR_LATENCY_5WS |  // 5 wait states для 168 MHz
                 FLASH_ACR_PRFTEN      |  // Включить prefetch
                 FLASH_ACR_ICEN        |  // Включить instruction cache
                 FLASH_ACR_DCEN;          // Включить data cache
    
    // Переключение на PLL как источника системной частоты
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    timeout = 1000000;
    while(((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) && timeout--) {
        __NOP();
    }
}
