/* SPDX-License-Identifier: Apache-2.0 */
/* Minimal HAL definitions for LL USB compatibility */

#ifndef STM32F4xx_HAL_DEF_H
#define STM32F4xx_HAL_DEF_H

#include <stdint.h>
#include <stddef.h>

typedef enum {
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

#define __IO volatile
#define __ALIGNED(x) __attribute__((aligned(x)))

#endif /* STM32F4xx_HAL_DEF_H */
