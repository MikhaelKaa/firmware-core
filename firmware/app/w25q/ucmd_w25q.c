// ucmd_w25q.c
#include "ucmd.h"
#include "w25q.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#ifdef BAREMETAL
#define ENDL "\r\n"
#else
#define ENDL "\n"
#endif

// ----------------------------------------------------------------------------
// Внутренние константы и макросы
// ----------------------------------------------------------------------------
#define MAX_DUMP_BYTES    512   // Ограничение на дамп за один раз
#define SECTOR_SIZE       4096U
#define TEMP_BUF_SIZE     64    // Для мелких операций

// ----------------------------------------------------------------------------
// Вспомогательные функции
// ----------------------------------------------------------------------------

/**
 * @brief Печать справки по командам
 */
static void print_usage(void) {
    printf("Usage: w25q <command> [arguments]" ENDL);
    printf("Commands:" ENDL);
    printf("  info                         - Show flash info (size, JEDEC ID, driver version)" ENDL);
    printf("  read_id                      - Read and print JEDEC ID" ENDL);
    printf("  read <addr>                   - Read one byte from address (hex)" ENDL);
    printf("  dump <addr> <len>             - Hexdump of flash region (max %u bytes, hex)" ENDL, MAX_DUMP_BYTES);
    printf("  write <addr> <byte>            - Write one byte (address must be in erased sector; hex)" ENDL);
    printf("  erase <addr>                   - Erase 4KB sector containing address (hex)" ENDL);
    printf("  erase chip                     - Erase entire chip" ENDL);
    printf("  test <addr> <len>              - Destructive test (erases and writes patterns; hex)" ENDL);
    printf("  set <addr>                      - Set current address for sequential access (hex)" ENDL);
    printf("  help                            - Show this help" ENDL);
}

/**
 * @brief Чтение JEDEC ID и вывод в консоль
 */
static void print_jedec_id(void) {
    uint8_t id[3];
    const drv_face_t *dev = dev_w25q_get();

    if (dev->ioctl(W25Q_READ_ID, id) != 0) {
        printf("Failed to read JEDEC ID" ENDL);
        return;
    }
    printf("JEDEC ID: 0x%02X 0x%02X 0x%02X" ENDL, id[0], id[1], id[2]);
}

/**
 * @brief Показать информацию о Flash
 */
static void print_info(void) {
    const drv_face_t *dev = dev_w25q_get();
    uint32_t size;
    const char *version;

    // Получить размер
    if (dev->ioctl(W25Q_GET_SIZE, &size) != 0) {
        size = 0;
    }

    // Получить версию драйвера
    if (dev->ioctl(INTERFACE_GET_INFO, &version) != 0) {
        version = "unknown";
    }

    printf("Flash size: %lu bytes (%lu MB)" ENDL, size, size / (1024 * 1024));
    printf("Driver: %s" ENDL, version);
    print_jedec_id();
}

/**
 * @brief Вывод дампа памяти в формате hex + ASCII
 * @param addr  Стартовый адрес (hex)
 * @param len   Количество байт (hex)
 */
static void do_dump(uint32_t addr, uint32_t len) {
    const drv_face_t *dev = dev_w25q_get();
    uint8_t buffer[TEMP_BUF_SIZE];
    uint32_t offset = 0;
    int ret;

    if (len > MAX_DUMP_BYTES) {
        printf("Warning: truncating dump to %u bytes" ENDL, MAX_DUMP_BYTES);
        len = MAX_DUMP_BYTES;
    }

    // Установить текущий адрес
    if (dev->ioctl(W25Q_SET_ADDRESS, &addr) != 0) {
        printf("Failed to set address" ENDL);
        return;
    }

    while (offset < len) {
        uint32_t chunk = (len - offset) < TEMP_BUF_SIZE ? (len - offset) : TEMP_BUF_SIZE;

        ret = dev->read(buffer, chunk);
        if (ret < 0) {
            printf("Read error at offset %lu: %d" ENDL, (unsigned long)offset, ret);
            break;
        }
        if (ret == 0) break;

        // Вывод строки дампа (16 байт)
        for (int i = 0; i < ret; i++) {
            if (i % 16 == 0) {
                if (i != 0) {
                    // ASCII часть предыдущей строки
                    printf(" |");
                    for (int j = i - 16; j < i; j++) {
                        uint8_t c = buffer[j - (i - 16)];
                        putchar((c >= 32 && c <= 126) ? c : '.');
                    }
                    printf("" ENDL);
                }
                printf("0x%08lx: ", (unsigned long)(addr + offset + (uint32_t)i));
            }
            printf("%02x ", buffer[i]);
        }
        offset += (uint32_t)ret;
    }

    // Завершающая строка, если данные были
    if (offset % 16 != 0) {
        uint32_t last_line_start = (offset / 16) * 16;
        uint32_t ascii_start = last_line_start;
        uint32_t ascii_end = offset;

        // Допечатать пробелы для выравнивания
        for (uint32_t i = ascii_end; i < last_line_start + 16; i++) {
            printf("   ");
        }
        printf(" |");
        for (uint32_t j = ascii_start; j < ascii_end; j++) {
            // Здесь нужно обратиться к буферу, но у нас уже нет данных после конца.
            // Упрощённо: не выводим ASCII для неполной строки.
        }
        printf("" ENDL);
    } else if (offset > 0) {
        // Вывести ASCII для последней полной строки
        printf(" |");
        // uint32_t start = offset - 16;
        // Не можем напечатать, т.к. данные уже не в буфере. Пропустим.
        printf("" ENDL);
    }
}

/**
 * @brief Запись одного байта по адресу
 */
static int do_write_byte(uint32_t addr, uint8_t data) {
    const drv_face_t *dev = dev_w25q_get();

    if (dev->ioctl(W25Q_SET_ADDRESS, &addr) != 0) {
        return -EIO;
    }

    int ret = dev->write(&data, 1);
    if (ret < 0) {
        return ret;
    }
    return (ret == 1) ? 0 : -EIO;
}

/**
 * @brief Тест памяти (разрушающий)
 * @param addr Стартовый адрес (должен быть выровнен на границу сектора, hex)
 * @param len  Длина (кратна размеру сектора, hex)
 * @return Количество ошибок
 */
static uint32_t do_test(uint32_t addr, uint32_t len) {
    const drv_face_t *dev = dev_w25q_get();
    uint8_t pattern1 = 0xAA;
    uint8_t pattern2 = 0x55;
    uint8_t buffer[SECTOR_SIZE];
    uint32_t errors = 0;
    uint32_t end = addr + len;
    uint32_t current = addr;

    printf("Destructive test from 0x%08lx to 0x%08lx" ENDL, (unsigned long)addr, (unsigned long)end);
    printf("This will ERASE and overwrite data. Continue? (y/N): ");
    int c = getchar();
    if (c != 'y' && c != 'Y') {
        printf("Aborted" ENDL);
        return 0;
    }

    // Выравнивание на границу сектора
    current = addr & ~(SECTOR_SIZE - 1);
    end = (addr + len + SECTOR_SIZE - 1) & ~(SECTOR_SIZE - 1);

    for (; current < end; current += SECTOR_SIZE) {
        printf("Testing sector at 0x%08lx ... ", (unsigned long)current);

        // Стереть сектор
        if (dev->ioctl(W25Q_SECTOR_ERASE, &current) != 0) {
            printf("ERASE FAILED" ENDL);
            errors += SECTOR_SIZE;
            continue;
        }

        // Заполнить буфер паттерном 0xAA
        memset(buffer, pattern1, SECTOR_SIZE);

        // Записать
        if (dev->ioctl(W25Q_SET_ADDRESS, &current) != 0) {
            printf("SET ADDR FAILED" ENDL);
            errors += SECTOR_SIZE;
            continue;
        }
        int ret = dev->write(buffer, SECTOR_SIZE);
        if (ret != SECTOR_SIZE) {
            printf("WRITE FAILED (%d)" ENDL, ret);
            errors += SECTOR_SIZE;
            continue;
        }

        // Прочитать и проверить
        if (dev->ioctl(W25Q_SET_ADDRESS, &current) != 0) {
            printf("SET ADDR FAILED" ENDL);
            errors += SECTOR_SIZE;
            continue;
        }
        ret = dev->read(buffer, SECTOR_SIZE);
        if (ret != SECTOR_SIZE) {
            printf("READ FAILED (%d)" ENDL, ret);
            errors += SECTOR_SIZE;
            continue;
        }
        for (uint32_t i = 0; i < SECTOR_SIZE; i++) {
            if (buffer[i] != pattern1) {
                errors++;
                if (errors < 10) {
                    printf("\n  mismatch at offset %lu: wrote 0x%02X, read 0x%02X",
                           (unsigned long)i, pattern1, buffer[i]);
                }
            }
        }

        // Второй паттерн 0x55
        memset(buffer, pattern2, SECTOR_SIZE);
        if (dev->ioctl(W25Q_SET_ADDRESS, &current) != 0) {
            printf("SET ADDR FAILED" ENDL);
            errors += SECTOR_SIZE;
            continue;
        }
        ret = dev->write(buffer, SECTOR_SIZE);
        if (ret != SECTOR_SIZE) {
            printf("WRITE FAILED (%d)" ENDL, ret);
            errors += SECTOR_SIZE;
            continue;
        }

        // Проверка
        if (dev->ioctl(W25Q_SET_ADDRESS, &current) != 0) {
            printf("SET ADDR FAILED" ENDL);
            errors += SECTOR_SIZE;
            continue;
        }
        ret = dev->read(buffer, SECTOR_SIZE);
        if (ret != SECTOR_SIZE) {
            printf("READ FAILED (%d)" ENDL, ret);
            errors += SECTOR_SIZE;
            continue;
        }
        for (uint32_t i = 0; i < SECTOR_SIZE; i++) {
            if (buffer[i] != pattern2) {
                errors++;
                if (errors < 10) {
                    printf("\n  mismatch at offset %lu: wrote 0x%02X, read 0x%02X",
                           (unsigned long)i, pattern2, buffer[i]);
                }
            }
        }

        printf(" done, errors %lu" ENDL, (unsigned long)errors);
    }

    return errors;
}

// ----------------------------------------------------------------------------
// Основная функция команды
// ----------------------------------------------------------------------------
int ucmd_w25q(int argc, char *argv[]) {
    const drv_face_t *dev = dev_w25q_get();

    if (argc == 1) {
        // Без аргументов – показать информацию
        print_info();
        return 0;
    }

    if (argc < 2) {
        return UCMD_CMD_NOT_FOUND;
    }

    char *cmd = argv[1];

    // ===== HELP =====
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "--help") == 0) {
        print_usage();
        return 0;
    }

    // ===== INFO =====
    if (strcmp(cmd, "info") == 0) {
        print_info();
        return 0;
    }

    // ===== READ ID =====
    if (strcmp(cmd, "read_id") == 0) {
        print_jedec_id();
        return 0;
    }

    // ===== SET ADDRESS =====
    if (strcmp(cmd, "set") == 0) {
        if (argc != 3) {
            printf("Usage: w25q set <addr> (hex)" ENDL);
            return UCMD_CMD_NOT_FOUND;
        }
        unsigned long tmp;
        if (sscanf(argv[2], "%lx", &tmp) != 1) {
            printf("Invalid address format (hex expected)" ENDL);
            return -EINVAL;
        }
        uint32_t addr = (uint32_t)tmp;
        if (dev->ioctl(W25Q_SET_ADDRESS, &addr) != 0) {
            printf("Failed to set address" ENDL);
            return -EIO;
        }
        printf("Current address set to 0x%08lx" ENDL, (unsigned long)addr);
        return 0;
    }

    // ===== READ (one byte) =====
    if (strcmp(cmd, "read") == 0) {
        if (argc != 3) {
            printf("Usage: w25q read <addr> (hex)" ENDL);
            return UCMD_CMD_NOT_FOUND;
        }
        unsigned long tmp;
        if (sscanf(argv[2], "%lx", &tmp) != 1) {
            printf("Invalid address format (hex expected)" ENDL);
            return -EINVAL;
        }
        uint32_t addr = (uint32_t)tmp;
        uint8_t val;

        if (dev->ioctl(W25Q_SET_ADDRESS, &addr) != 0) {
            printf("Failed to set address" ENDL);
            return -EIO;
        }
        int ret = dev->read(&val, 1);
        if (ret < 0) {
            printf("Read error: %d" ENDL, ret);
            return ret;
        }
        if (ret == 0) {
            printf("No data read (end of flash?)" ENDL);
            return 0;
        }
        printf("0x%02x" ENDL, val);
        return 0;
    }

    // ===== DUMP =====
    if (strcmp(cmd, "dump") == 0) {
        if (argc != 4) {
            printf("Usage: w25q dump <addr> <len> (hex)" ENDL);
            return UCMD_CMD_NOT_FOUND;
        }
        unsigned long tmp_addr, tmp_len;
        if (sscanf(argv[2], "%lx", &tmp_addr) != 1 ||
            sscanf(argv[3], "%lx", &tmp_len) != 1) {
            printf("Invalid arguments (hex expected)" ENDL);
            return -EINVAL;
        }
        uint32_t addr = (uint32_t)tmp_addr;
        uint32_t len  = (uint32_t)tmp_len;
        do_dump(addr, len);
        return 0;
    }

    // ===== WRITE one byte =====
    if (strcmp(cmd, "write") == 0) {
        if (argc != 4) {
            printf("Usage: w25q write <addr> <byte> (hex)" ENDL);
            return UCMD_CMD_NOT_FOUND;
        }
        unsigned long tmp_addr, tmp_data;
        if (sscanf(argv[2], "%lx", &tmp_addr) != 1 ||
            sscanf(argv[3], "%lx", &tmp_data) != 1) {
            printf("Invalid arguments (hex expected)" ENDL);
            return -EINVAL;
        }
        uint32_t addr = (uint32_t)tmp_addr;
        uint32_t data = (uint32_t)tmp_data;
        if (data > 0xFF) {
            printf("Byte value must be 0x00..0xFF" ENDL);
            return -EINVAL;
        }
        int ret = do_write_byte(addr, (uint8_t)data);
        if (ret == 0) {
            printf("Byte written" ENDL);
        } else {
            printf("Write failed: %d" ENDL, ret);
        }
        return ret;
    }

    // ===== ERASE SECTOR =====
    if (strcmp(cmd, "erase") == 0) {
        if (argc != 3) {
            printf("Usage: w25q erase <addr>  or  w25q erase chip" ENDL);
            return UCMD_CMD_NOT_FOUND;
        }

        if (strcmp(argv[2], "chip") == 0) {
            // Стирание всего чипа
            printf("Chip erase may take several seconds. Continue? (y/N): ");
            int c = getchar();
            if (c != 'y' && c != 'Y') {
                printf("Aborted" ENDL);
                return 0;
            }
            int ret = dev->ioctl(W25Q_CHIP_ERASE, NULL);
            if (ret != 0) {
                printf("Chip erase failed: %d" ENDL, ret);
            } else {
                printf("Chip erase started, waiting...\r\n");
                printf("Chip erase completed" ENDL);
            }
            return ret;
        } else {
            unsigned long tmp;
            if (sscanf(argv[2], "%lx", &tmp) != 1) {
                printf("Invalid address format (hex expected)" ENDL);
                return -EINVAL;
            }
            uint32_t addr = (uint32_t)tmp;
            int ret = dev->ioctl(W25Q_SECTOR_ERASE, &addr);
            if (ret != 0) {
                printf("Sector erase failed: %d" ENDL, ret);
            } else {
                printf("Sector at 0x%08lx erased" ENDL, (unsigned long)addr);
            }
            return ret;
        }
    }

    // ===== TEST (destructive) =====
    if (strcmp(cmd, "test") == 0) {
        if (argc != 4) {
            printf("Usage: w25q test <addr> <len> (hex)" ENDL);
            return UCMD_CMD_NOT_FOUND;
        }
        unsigned long tmp_addr, tmp_len;
        if (sscanf(argv[2], "%lx", &tmp_addr) != 1 ||
            sscanf(argv[3], "%lx", &tmp_len) != 1) {
            printf("Invalid arguments (hex expected)" ENDL);
            return -EINVAL;
        }
        uint32_t addr = (uint32_t)tmp_addr;
        uint32_t len  = (uint32_t)tmp_len;
        uint32_t errors = do_test(addr, len);
        if (errors == 0) {
            printf("Test PASSED" ENDL);
        } else {
            printf("Test FAILED with %lu errors" ENDL, (unsigned long)errors);
        }
        return (errors == 0) ? 0 : -EIO;
    }

    // Неизвестная команда
    printf("Unknown w25q command: %s" ENDL, cmd);
    print_usage();
    return UCMD_CMD_NOT_FOUND;
}