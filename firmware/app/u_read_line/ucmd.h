#ifndef _UCMD_H_
#define _UCMD_H_

#include <limits.h>

// Ошибка, если команда не найдена
#define UCMD_CMD_NOT_FOUND INT_MIN

// Тип функции обратного вызова для команды
typedef int (*command_cb)(int, char **);

// Структура описания команды
typedef struct command {
    const char *cmd;    /**< the command string to match against */
    const char *help;   /**< the help text associated with cmd */
    command_cb fn;      /**< the function to call when cmd is matched */
} command_t;

// Инициализация обработчика команд по умолчанию
void ucmd_default_init(void);

// Вызов в основном цикле для обработки входных данных
void ucmd_default_proc(void);

// Парсинг команд и их выполнение
int ucmd_parse(command_t[], int argc, const char **argv);

// Функция обратного вызова для вывода помощи
int print_help_cb(int argc, char *argv[]);

// Функция вывода строки в обработчике команд
void ucmd_default_print(const char * str);

// Установка функции обработки прерывания SIGINT
void ucmd_set_sigint(void (*sigintf)(void));

#endif /* _UCMD_H_ */
