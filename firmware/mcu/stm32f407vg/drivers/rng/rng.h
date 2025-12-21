/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

#ifndef DEV_RNG_H
#define DEV_RNG_H

#include "dev_interface.h"
#include <stdint.h>

// RNG-specific ioctl commands
#define RNG_INIT       (INTERFACE_CMD_DEVICE + 0)
#define RNG_DEINIT     (INTERFACE_CMD_DEVICE + 1)
#define RNG_GET_STATUS (INTERFACE_CMD_DEVICE + 2)
#define RNG_RESET      (INTERFACE_CMD_DEVICE + 3)
#define RNG_SELF_TEST  (INTERFACE_CMD_DEVICE + 4)

// RNG status flags
#define RNG_STATUS_READY       0x00000001
#define RNG_STATUS_ERROR       0x00000002
#define RNG_STATUS_SEED_ERROR  0x00000004
#define RNG_STATUS_CLOCK_ERROR 0x00000008

// RNG configuration
#define RNG_TIMEOUT            10000  // Timeout for RNG operations

// RNG device instance
const interface_t* dev_rng_get(void);

#endif /* DEV_RNG_H */