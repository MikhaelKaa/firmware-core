// ucmd_led.c
#include "ucmd.h"
#include "pwm_led.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// Предполагаем, что у нас есть глобальный экземпляр драйвера,
// объявленный в pwm_led_drv.h как extern const drv_face_t pwm_led_dev.
// Если нужно получать драйвер через таблицу, можно использовать соответствующий механизм.

// Текущее состояние LED (сохраняем, чтобы показывать по запросу)
static struct {
    led_mode_t mode;
    uint16_t   period_ms;
    uint8_t    min_bright;
    uint8_t    max_bright;
} led_state = {
    .mode = LED_MODE_OFF,
    .period_ms = 0,
    .min_bright = 0,
    .max_bright = 0
};

// Вывод текущих параметров LED
static void print_status(void) {
    const char *mode_str;

    switch (led_state.mode) {
        case LED_MODE_OFF:     mode_str = "OFF"; break;
        case LED_MODE_ON:      mode_str = "ON"; break;
        case LED_MODE_BLINK:   mode_str = "BLINK"; break;
        case LED_MODE_BREATHE: mode_str = "BREATHE"; break;
        case LED_MODE_FADE_IN: mode_str = "FADE_IN"; break;
        case LED_MODE_FADE_OUT:mode_str = "FADE_OUT"; break;
        default:               mode_str = "UNKNOWN"; break;
    }

    printf("LED mode: %s\r\n", mode_str);
    if (led_state.mode != LED_MODE_OFF && led_state.mode != LED_MODE_ON) {
        printf("  period: %u ms\r\n", led_state.period_ms);
        printf("  min brightness: %u\r\n", led_state.min_bright);
        printf("  max brightness: %u\r\n", led_state.max_bright);
    } else if (led_state.mode == LED_MODE_ON) {
        printf("  brightness: %u\r\n", led_state.max_bright); // для ON яркость = max
    }
}

// Проверка и обновление состояния с использованием нового драйвера
static void set_led_mode(led_mode_t mode, uint16_t period_ms, uint8_t min_bright, uint8_t max_bright) {
    led_state.mode = mode;
    led_state.period_ms = period_ms;
    led_state.min_bright = min_bright;
    led_state.max_bright = max_bright;

    // Вызов inline-обёртки для нового драйвера
    pwm_led_set_mode((drv_face_t*)&pwm_led_dev, mode, period_ms, min_bright, max_bright);
}

int ucmd_led(int argc, char *argv[]) {
    // Без аргументов – показать состояние
    if (argc == 1) {
        print_status();
        return 0;
    }

    // Первый аргумент – подкоманда
    if (argc < 2) {
        return UCMD_CMD_NOT_FOUND; // Не должно случиться
    }

    char *cmd = argv[1];

    // ===== OFF =====
    if (strcmp(cmd, "off") == 0) {
        set_led_mode(LED_MODE_OFF, 0, 0, 0);
        printf("LED off\r\n");
        return 0;
    }

    // ===== ON <brightness> =====
    if (strcmp(cmd, "on") == 0) {
        if (argc != 3) {
            printf("Usage: led on <brightness(0-255)>\r\n");
            return UCMD_CMD_NOT_FOUND;
        }
        int bright = atoi(argv[2]);
        if (bright < 0 || bright > 255) {
            printf("Brightness must be 0..255\r\n");
            return UCMD_CMD_NOT_FOUND;
        }
        set_led_mode(LED_MODE_ON, 0, (uint8_t)bright, (uint8_t)bright);
        printf("LED on, brightness=%d\r\n", bright);
        return 0;
    }

    // ===== BLINK <period_ms> <min> <max> =====
    if (strcmp(cmd, "blink") == 0) {
        if (argc != 5) {
            printf("Usage: led blink <period_ms> <min_bright(0-255)> <max_bright(0-255)>\r\n");
            return UCMD_CMD_NOT_FOUND;
        }
        int period = atoi(argv[2]);
        int min    = atoi(argv[3]);
        int max    = atoi(argv[4]);
        if (period <= 0 || period > 65535) {
            printf("Period must be 1..65535 ms\r\n");
            return UCMD_CMD_NOT_FOUND;
        }
        if (min < 0 || min > 255 || max < 0 || max > 255) {
            printf("Brightness values must be 0..255\r\n");
            return UCMD_CMD_NOT_FOUND;
        }
        set_led_mode(LED_MODE_BLINK, (uint16_t)period, (uint8_t)min, (uint8_t)max);
        printf("LED blink set: period=%u ms, min=%u, max=%u\r\n", period, min, max);
        return 0;
    }

    // ===== BREATHE <period_ms> <min> <max> =====
    if (strcmp(cmd, "breathe") == 0) {
        if (argc != 5) {
            printf("Usage: led breathe <period_ms> <min_bright(0-255)> <max_bright(0-255)>\r\n");
            return UCMD_CMD_NOT_FOUND;
        }
        int period = atoi(argv[2]);
        int min    = atoi(argv[3]);
        int max    = atoi(argv[4]);
        if (period <= 0 || period > 65535) {
            printf("Period must be 1..65535 ms\r\n");
            return UCMD_CMD_NOT_FOUND;
        }
        if (min < 0 || min > 255 || max < 0 || max > 255) {
            printf("Brightness values must be 0..255\r\n");
            return UCMD_CMD_NOT_FOUND;
        }
        set_led_mode(LED_MODE_BREATHE, (uint16_t)period, (uint8_t)min, (uint8_t)max);
        printf("LED breathe set: period=%u ms, min=%u, max=%u\r\n", period, min, max);
        return 0;
    }

    // ===== FADEIN <period_ms> <min> <max> =====
    if (strcmp(cmd, "fadein") == 0) {
        if (argc != 5) {
            printf("Usage: led fadein <period_ms> <min_bright(0-255)> <max_bright(0-255)>\r\n");
            return UCMD_CMD_NOT_FOUND;
        }
        int period = atoi(argv[2]);
        int min    = atoi(argv[3]);
        int max    = atoi(argv[4]);
        if (period <= 0 || period > 65535) {
            printf("Period must be 1..65535 ms\r\n");
            return UCMD_CMD_NOT_FOUND;
        }
        if (min < 0 || min > 255 || max < 0 || max > 255) {
            printf("Brightness values must be 0..255\r\n");
            return UCMD_CMD_NOT_FOUND;
        }
        set_led_mode(LED_MODE_FADE_IN, (uint16_t)period, (uint8_t)min, (uint8_t)max);
        printf("LED fade-in set: period=%u ms, min=%u, max=%u\r\n", period, min, max);
        return 0;
    }

    // ===== FADEOUT <period_ms> <min> <max> =====
    if (strcmp(cmd, "fadeout") == 0) {
        if (argc != 5) {
            printf("Usage: led fadeout <period_ms> <min_bright(0-255)> <max_bright(0-255)>\r\n");
            return UCMD_CMD_NOT_FOUND;
        }
        int period = atoi(argv[2]);
        int min    = atoi(argv[3]);
        int max    = atoi(argv[4]);
        if (period <= 0 || period > 65535) {
            printf("Period must be 1..65535 ms\r\n");
            return UCMD_CMD_NOT_FOUND;
        }
        if (min < 0 || min > 255 || max < 0 || max > 255) {
            printf("Brightness values must be 0..255\r\n");
            return UCMD_CMD_NOT_FOUND;
        }
        set_led_mode(LED_MODE_FADE_OUT, (uint16_t)period, (uint8_t)min, (uint8_t)max);
        printf("LED fade-out set: period=%u ms, min=%u, max=%u\r\n", period, min, max);
        return 0;
    }

    // Если команда не распознана
    printf("Unknown LED command: %s\r\n", cmd);
    printf("Available commands:\r\n");
    printf("  led                         - show status\r\n");
    printf("  off                         - turn off\r\n");
    printf("  on <bright>                  - steady on\r\n");
    printf("  blink <period> <min> <max>   - blinking\r\n");
    printf("  breathe <period> <min> <max> - breathing effect\r\n");
    printf("  fadein <period> <min> <max>  - fade in then stay\r\n");
    printf("  fadeout <period> <min> <max> - fade out then stay\r\n");
    return UCMD_CMD_NOT_FOUND;
}