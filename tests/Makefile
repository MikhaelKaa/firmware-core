# Makefile для юнит-тестов firmware-core (хост-компиляция, gcc x86_64)
#
# Использование:
#   make          - собрать и запустить все тесты
#   make clean    - удалить артефакты

CC      = gcc
CFLAGS  = -std=c99 -Wall -Wextra -Wpedantic -Werror
CFLAGS += -g -O0
CFLAGS += -D_DEFAULT_SOURCE

# Unity (локальная копия, позже заменится на submodule)
UNITY_DIR = unity/src

# Пути к стубам и тестируемым модулям
STUBS_DIR    = test_helpers
LIBS_DIR     = lib

# ------------------------------------------------------------------
# Include-пути
# ВАЖНО: test_helpers/ идёт первым, чтобы cmsis_stubs.h перекрыл
# оригинальный stm32f407xx.h из CMSIS.
# ------------------------------------------------------------------
INCLUDES = -I$(STUBS_DIR)
INCLUDES += -I$(UNITY_DIR)
INCLUDES += -I../lib/time

# ------------------------------------------------------------------
# Исходники
# ------------------------------------------------------------------

ALL_SRCS = $(STUBS_DIR)/stubs.c \
           $(UNITY_DIR)/unity.c \
           $(wildcard $(LIBS_DIR)/*/test_*.c)

TARGET = test_runner

# ------------------------------------------------------------------
# Правила
# ------------------------------------------------------------------

all: $(TARGET)
	@echo ""
	@echo "=== Запуск тестов ==="
	./$(TARGET)

$(TARGET): $(ALL_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -o $@

clean:
	rm -f $(TARGET)

.PHONY: all clean