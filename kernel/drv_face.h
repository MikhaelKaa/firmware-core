/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#ifndef _DRV_FACE_H
#define _DRV_FACE_H

#include <stddef.h>
#include <stdint.h>
#include <errno.h>
#include <stdio.h>

// Общие команды ioctl
#define INTERFACE_GET_INFO   0x1000 /* Получение информации о устройстве */
#define INTERFACE_INIT       0x1001 /* Инициализация устройства */
#define INTERFACE_DEINIT     0x1002 /* Деинициализация устройства */
#define INTERFACE_GET_PROC   0x1003 /* Получение функции обработки устройства */

// База для команд, специфичных для устройства
#define INTERFACE_CMD_DEVICE 0x2000 /* Базовый номер для устройствозависимых команд */

/**
 * struct drv_face - Общий интерфейс устройства
 * @read:  Чтение данных из устройства
 * @write: Запись данных в устройство
 * @ioctl: Управление и конфигурация устройства
 *
 * Общий интерфейс для всех типов устройств в системе.
 * Функции должны возвращать 0 при успехе или отрицательное значение errno при ошибке.
 */
typedef struct drv_face
{
    int (*read)(void* buf, size_t len);
    int (*write)(const void* buf, size_t len);
    int (*ioctl)(int cmd, void* arg);
} drv_face_t;

// Установка устройства в таблицу драйверов
ssize_t drv_table_set(const drv_face_t* dev, unsigned int pos);

// Получение устройства из таблицы драйверов
ssize_t drv_table_get(drv_face_t** dev, unsigned int pos);

#endif /* _DRV_FACE_H */
