/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

#ifndef DEV_ADC1_H
#define DEV_ADC1_H

#include <stddef.h>
#include <stdint.h>
#include "drv_face.h"

/* ADC1-specific ioctl commands */
#define ADC1_GET_AVAILABLE    (INTERFACE_CMD_DEVICE + 0)  // Number of available samples in buffer
#define ADC1_SET_FREQ         (INTERFACE_CMD_DEVICE + 1)  // Set sampling frequency (uint32_t*)
#define ADC1_SET_CB_HALF      (INTERFACE_CMD_DEVICE + 2)  // Set half-buffer callback
#define ADC1_SET_CB_FULL      (INTERFACE_CMD_DEVICE + 3)  // Set full-buffer callback
#define ADC1_GET_STATUS       (INTERFACE_CMD_DEVICE + 4)  // Get ADC1 status (uint32_t*)
#define ADC1_START            (INTERFACE_CMD_DEVICE + 5)  // Start ADC conversion
#define ADC1_STOP             (INTERFACE_CMD_DEVICE + 6)  // Stop ADC conversion

/* ADC1 status flags */
#define ADC1_STATUS_RUNNING   0x00000001
#define ADC1_STATUS_READY     0x00000002
#define ADC1_STATUS_CALIBRATED 0x00000004

/* Buffer size */
#ifndef ADC1_BUFFER_SIZE
#define ADC1_BUFFER_SIZE      2048U
#endif

/* Default sampling frequency (Hz) */
#ifndef ADC1_DEFAULT_FREQ
#define ADC1_DEFAULT_FREQ     10000U
#endif

/* ADC1 device instance */
const drv_face_t* dev_adc1_get(void);

#endif /* DEV_ADC1_H */