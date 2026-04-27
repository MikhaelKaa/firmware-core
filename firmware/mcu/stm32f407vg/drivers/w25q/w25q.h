/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

#ifndef DEV_W25Q_H
#define DEV_W25Q_H

#include <stddef.h>
#include <stdint.h>
#include "drv_face.h"

// Команды ioctl для управления W25Q флеш-памятью
#define W25Q_SET_ADDRESS    (INTERFACE_CMD_DEVICE + 0)  // Установка адреса в памяти
#define W25Q_GET_SIZE       (INTERFACE_CMD_DEVICE + 1)  // Получение размера памяти
#define W25Q_READ_ID        (INTERFACE_CMD_DEVICE + 2)  // Чтение идентификатора устройства
#define W25Q_SECTOR_ERASE   (INTERFACE_CMD_DEVICE + 3)  // Стирание сектора
#define W25Q_CHIP_ERASE     (INTERFACE_CMD_DEVICE + 4)  // Массовое стирание памяти

// Получение экземпляра устройства W25Q флеш-памяти
const drv_face_t* dev_w25q_get(void);

#endif /* DEV_W25Q_H */
