/*
 * SPDX-FileCopyrightText: 2016-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//  iecc_tcp_slave.h Modbus controller TCP slave implementation header file

#pragma once

#include <stdint.h>                 // for standard int types definition
#include <stddef.h>                 // for NULL and std defines
#include "esp_iec_common.h"      // for common defines

/* ----------------------- Defines ------------------------------------------*/

#define IEC_CONTROLLER_NOTIFY_QUEUE_SIZE     (CONFIG_FIEC_CONTROLLER_NOTIFY_QUEUE_SIZE) // Nuiecer of messages in parameter notification queue
#define IEC_CONTROLLER_NOTIFY_TIMEOUT        (pdMS_TO_TICKS(CONFIG_FIEC_CONTROLLER_NOTIFY_TIMEOUT)) // notification timeout

/**
 * @brief Initialize Modbus controller and stack for TCP slave
 *
 * @param[out] ctx - pointer to pointer of interface structure
 * @param[in] config - pointer to configuration structure
 * @return
 *     - ESP_OK   Success
 *     - ESP_ERR_NO_MEM Parameter error
 */
esp_err_t iecc_tcp_slave_create(iec_communication_info_t *config, void **ctx);
