/*
 * SPDX-FileCopyrightText: 2016-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"                    // for esp_err_t
#include "sdkconfig.h"                  // for KConfig defines
#include "iecc_slave.h"                  // for slave interface define
#include "esp_iec_slave.h"               // for public slave defines
#include "iecc_serial_slave.h"           // for public interface defines
#include "iec_port_types.h"

/**
 * Initialization of Modbus Serial slave controller
 */
esp_err_t iecc_slave_create_serial(iec_communication_info_t *config, void **handler)
{
    void *ctx = NULL;
    esp_err_t error = iecc_serial_slave_create(config, &ctx);
    if ((ctx) && (error == ESP_OK))
        *handler = ctx;
    return error;
}