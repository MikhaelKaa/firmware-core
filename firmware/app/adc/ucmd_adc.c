// ucmd_adc.c - CLI utility for ADC testing
// Interacts with ADC driver ONLY through drv_face_t interface

#include "ucmd.h"
#include "adc.h"
#include "drv_face.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef BAREMETAL
#define ENDL "\r\n"
#else
#define ENDL "\n"
#endif

/* Maximum samples for single dump */
#define ADC_MAX_DUMP_SAMPLES  256

/* Temporary buffer for reading (512 bytes = 256 samples) */
#define ADC_TEMP_BUF_SIZE     512

/* Maximum wait time in milliseconds (approximate) */
#define ADC_MAX_WAIT_MS       5000

/**
 * @brief Печать справки по командам ADC
 */
static void print_usage(void) {
    printf("Usage: adc <command> [arguments]" ENDL);
    printf("Commands:" ENDL);
    printf("  start [freq]           - Start ADC sampling (freq in Hz, default: %d)" ENDL, ADC1_DEFAULT_FREQ);
    printf("  stop                   - Stop ADC sampling" ENDL);
    printf("  wait [ms]              - Wait for data (default: 1000 ms)" ENDL);
    printf("  dump [samples]         - Dump last N samples as hex memory dump (default: 256)" ENDL);
    printf("  status                 - Show ADC1 status" ENDL);
    printf("  help                   - Show this help" ENDL);
}

/**
 * @brief Начать сборку ADC через интерфейс драйвера
 */
static int do_start(uint32_t freq) {
    const drv_face_t *dev = dev_adc1_get();
    int ret;

    printf("Starting ADC1 at %lu Hz..." ENDL, (unsigned long)freq);

    /* Set frequency */
    ret = dev->ioctl(ADC1_SET_FREQ, &freq);
    if (ret != 0) {
        printf("Failed to set frequency: %d" ENDL, ret);
        return ret;
    }

    /* Start ADC */
    ret = dev->ioctl(ADC1_START, NULL);
    if (ret != 0) {
        printf("Failed to start ADC: %d" ENDL, ret);
        return ret;
    }

    printf("ADC1 started successfully" ENDL);
    return 0;
}

/**
 * @brief Остановить сборку ADC через интерфейс драйвера
 */
static int do_stop(void) {
    const drv_face_t *dev = dev_adc1_get();
    int ret;

    printf("Stopping ADC1..." ENDL);

    ret = dev->ioctl(ADC1_STOP, NULL);
    if (ret != 0) {
        printf("Failed to stop ADC: %d" ENDL, ret);
        return ret;
    }

    printf("ADC1 stopped" ENDL);
    return 0;
}

/**
 * @brief Ожидание появления данных от ADC через интерфейс драйвера
 */
static int do_wait(uint32_t timeout_ms) {
    const drv_face_t *dev = dev_adc1_get();
    uint32_t elapsed = 0;
    uint32_t step = 10;
    uint32_t available = 0;
    int ret;

    printf("Waiting for data (timeout: %lu ms)..." ENDL, (unsigned long)timeout_ms);

    while (elapsed < timeout_ms) {
        ret = dev->ioctl(ADC1_GET_AVAILABLE, &available);
        if (ret == 0 && available > 0) {
            printf("Got %lu samples after %lu ms" ENDL, (unsigned long)available, (unsigned long)elapsed);
            return 0;
        }
        for (volatile int i = 0; i < 10000; i++);
        elapsed += step;
    }

    printf("Timeout waiting for data" ENDL);
    return -ETIMEDOUT;
}

/**
 * @brief Вывод дампа памяти в формате hex dump (как mem dump)
 * @param buf          Указатель на начало области памяти
 * @param len          Количество байт
 * @param base_addr    Базовый адрес для отображения (0 если неизвестен)
 */
static void adc_mem_dump(uint8_t *buf, uint32_t len, uint32_t base_addr) {
    uint32_t i = 0;
    uint32_t current_addr = base_addr;
    const uint32_t bytes_per_line = 16;

    while (i < len) {
        printf("0x%08lx: ", (unsigned long)current_addr);

        uint32_t bytes_printed = 0;
        for (uint32_t j = 0; j < bytes_per_line; j++) {
            if (i + j < len) {
                printf("%02x ", buf[i + j]);
                bytes_printed++;
            } else {
                printf("   ");
            }
        }

        printf("|");
        for (uint32_t j = 0; j < bytes_printed; j++) {
            uint8_t c = buf[i + j];
            putchar((c >= 32 && c <= 126) ? c : '.');
        }
        printf("|" ENDL);

        i += bytes_printed;
        current_addr += bytes_printed;
    }
}

/**
 * @brief Вывод последних N отсчетов ADC через интерфейс драйвера
 */
static int do_dump(uint32_t num_samples) {
    const drv_face_t *dev = dev_adc1_get();
    uint32_t available = 0;
    uint32_t samples_to_read;
    uint8_t temp_buf[ADC_TEMP_BUF_SIZE];
    int ret;
    uint32_t bytes_read;
    uint32_t elapsed = 0;
    uint32_t step = 10;

    /* Ограничиваем количество образцов */
    if (num_samples == 0 || num_samples > ADC_MAX_DUMP_SAMPLES) {
        num_samples = ADC_MAX_DUMP_SAMPLES;
    }

    /* Получаем количество доступных образцов через интерфейс */
    ret = dev->ioctl(ADC1_GET_AVAILABLE, &available);
    if (ret != 0) {
        printf("Failed to get available count: %d" ENDL, ret);
        return ret;
    }

    /* Если данных нет, ждем до 5 секунд */
    if (available == 0) {
        printf("No data yet, waiting up to %d ms..." ENDL, ADC_MAX_WAIT_MS);
        while (elapsed < ADC_MAX_WAIT_MS) {
            ret = dev->ioctl(ADC1_GET_AVAILABLE, &available);
            if (ret == 0 && available > 0) {
                printf("Got %lu samples after %lu ms" ENDL, (unsigned long)available, (unsigned long)elapsed);
                break;
            }
            for (volatile int i = 0; i < 10000; i++);
            elapsed += step;
        }

        if (available == 0) {
            printf("Timeout waiting for samples" ENDL);
            return -ETIMEDOUT;
        }
    }

    /* Определяем сколько образцов читать */
    samples_to_read = available < num_samples ? available : num_samples;
    uint32_t bytes_to_read = samples_to_read * sizeof(uint16_t);
    if (bytes_to_read > ADC_TEMP_BUF_SIZE) {
        bytes_to_read = ADC_TEMP_BUF_SIZE;
    }

    /* Читаем данные через интерфейс драйвера */
    bytes_read = (uint32_t)dev->read(temp_buf, bytes_to_read);
    if (bytes_read == 0) {
        printf("No data read from driver" ENDL);
        return -EAGAIN;
    }

    printf("=== ADC1 DMA Buffer Dump ===" ENDL);
    printf("Samples: %lu (of %lu available)" ENDL, (unsigned long)(bytes_read / 2), (unsigned long)available);
    printf("-----||-------||-----------" ENDL);

    /* Выводим дамп */
    adc_mem_dump(temp_buf, bytes_read, 0);

    printf("-----||-------||-----------" ENDL);

    return 0;
}

/**
 * @brief Показать статус ADC1 через интерфейс драйвера
 */
static int do_status(void) {
    const drv_face_t *dev = dev_adc1_get();
    uint32_t status;
    uint32_t available;
    int ret;

    ret = dev->ioctl(ADC1_GET_STATUS, &status);
    if (ret != 0) {
        printf("Failed to get status: %d" ENDL, ret);
        return ret;
    }

    ret = dev->ioctl(ADC1_GET_AVAILABLE, &available);
    if (ret != 0) {
        available = 0;
    }

    printf("=== ADC1 Status ===" ENDL);
    printf("Running:    %s" ENDL, (status & ADC1_STATUS_RUNNING) ? "YES" : "NO");
    printf("Ready:      %s" ENDL, (status & ADC1_STATUS_READY) ? "YES" : "NO");
    printf("Calibrated: %s" ENDL, (status & ADC1_STATUS_CALIBRATED) ? "YES" : "NO");
    printf("Available samples: %lu" ENDL, (unsigned long)available);

    return 0;
}

/**
 * @brief Основная функция команды ADC
 */
int ucmd_adc(int argc, char *argv[]) {
    if (argc == 1) {
        print_usage();
        return 0;
    }

    if (argc < 2) {
        return UCMD_CMD_NOT_FOUND;
    }

    const char *cmd = argv[1];

    /* ===== HELP ===== */
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "--help") == 0) {
        print_usage();
        return 0;
    }

    /* ===== START ===== */
    if (strcmp(cmd, "start") == 0) {
        uint32_t freq = ADC1_DEFAULT_FREQ;
        if (argc >= 3) {
            char *endptr;
            unsigned long f = strtoul(argv[2], &endptr, 10);
            if (endptr == argv[2] || *endptr != '\0' || f == 0) {
                printf("Invalid frequency: %s (must be positive integer)" ENDL, argv[2]);
                return -EINVAL;
            }
            freq = (uint32_t)f;
        }
        return do_start(freq);
    }

    /* ===== STOP ===== */
    if (strcmp(cmd, "stop") == 0) {
        return do_stop();
    }

    /* ===== WAIT ===== */
    if (strcmp(cmd, "wait") == 0) {
        uint32_t timeout = 1000;
        if (argc >= 3) {
            char *endptr;
            unsigned long t = strtoul(argv[2], &endptr, 10);
            if (endptr == argv[2] || *endptr != '\0' || t == 0) {
                printf("Invalid timeout: %s (must be positive integer in ms)" ENDL, argv[2]);
                return -EINVAL;
            }
            timeout = (uint32_t)t;
        }
        return do_wait(timeout);
    }

    /* ===== DUMP ===== */
    if (strcmp(cmd, "dump") == 0) {
        uint32_t num_samples = ADC_MAX_DUMP_SAMPLES;
        if (argc >= 3) {
            char *endptr;
            unsigned long n = strtoul(argv[2], &endptr, 10);
            if (endptr == argv[2] || *endptr != '\0' || n == 0) {
                printf("Invalid sample count: %s (must be positive integer)" ENDL, argv[2]);
                return -EINVAL;
            }
            num_samples = (uint32_t)n;
        }
        return do_dump(num_samples);
    }

    /* ===== STATUS ===== */
    if (strcmp(cmd, "status") == 0) {
        return do_status();
    }

    /* Неизвестная команда */
    printf("Unknown adc command: %s" ENDL, cmd);
    print_usage();
    return UCMD_CMD_NOT_FOUND;
}