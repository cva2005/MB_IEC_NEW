/*
 * SPDX-FileCopyrightText: 2016-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"                // for esp_err_t
#include "esp_timer.h"              // for esp_timer_get_time()
#include "sdkconfig.h"              // for KConfig defines
#include "iecc_slave.h"              // for slave private type definitions

static const char TAG[] __attribute__((unused)) = "IEC_CONTROLLER_SLAVE";

/**
 * Critical section lock function
 */
esp_err_t iecc_slave_lock(void *ctx)
{
    IEC_RETURN_ON_FALSE(ctx, ESP_ERR_INVALID_STATE, TAG,
                            "Slave interface is not correctly initialized.");
    iecs_controller_iface_t *iecs_controller = IEC_SLAVE_GET_IFACE(ctx);
    iec_base_t *piec_obj = (iec_base_t *)iecs_controller->iec_base;
    IEC_RETURN_ON_FALSE((piec_obj && piec_obj->lock), ESP_ERR_INVALID_STATE, TAG,
                            "Slave interface is not correctly initialized.");
    CRITICAL_SECTION_LOCK(piec_obj->lock);
    return ESP_OK;
}

/**
 * Critical section unlock function
 */
esp_err_t iecc_slave_unlock(void *ctx)
{
    IEC_RETURN_ON_FALSE(ctx, ESP_ERR_INVALID_STATE, TAG,
                            "Slave interface is not correctly initialized.");
    iecs_controller_iface_t *iecs_controller = IEC_SLAVE_GET_IFACE(ctx);
    iec_base_t *piec_obj = (iec_base_t *)iecs_controller->iec_base;
    IEC_RETURN_ON_FALSE((piec_obj && piec_obj->lock), ESP_ERR_INVALID_STATE, TAG,
                            "Slave interface is not correctly initialized.");
    CRITICAL_SECTION_UNLOCK(piec_obj->lock);
    return ESP_OK;
}

/**
 * Start controller function
 */
esp_err_t iecc_slave_start(void *ctx)
{
    esp_err_t error = ESP_OK;
    IEC_RETURN_ON_FALSE(ctx, ESP_ERR_INVALID_STATE, TAG,
                        "Slave interface is not correctly initialized.");
    iecs_controller_iface_t *iecs_controller = IEC_SLAVE_GET_IFACE(ctx);
    IEC_RETURN_ON_FALSE(iecs_controller->start, ESP_ERR_INVALID_STATE, TAG,
                    "Slave interface is not correctly configured.");
    error = iecs_controller->start(ctx);
    IEC_RETURN_ON_FALSE((error == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                    "Slave start failure error=(0x%x).", (uint16_t)error);
    iecs_controller->is_active = true;
    return error;
}

/**
 * Controller stop function
 */
esp_err_t iecc_slave_stop(void *ctx)
{
    esp_err_t error = ESP_OK;
    IEC_RETURN_ON_FALSE(ctx, ESP_ERR_INVALID_STATE, TAG,
                        "Slave interface is not correctly initialized.");
    iecs_controller_iface_t *iecs_controller = IEC_SLAVE_GET_IFACE(ctx);
    IEC_RETURN_ON_FALSE(iecs_controller->stop,
                        ESP_ERR_INVALID_STATE, TAG,
                        "Slave interface is not correctly configured.");
    error = iecs_controller->stop(ctx);
    IEC_RETURN_ON_FALSE((error == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                    "Slave stop failure error=(0x%x).", (uint16_t)error);
    iecs_controller->is_active = false;
    return error;
}

/**
 * Blocking function to get event on parameter group change for application task
 */
iec_event_group_t iecc_slave_check_event(void *ctx, iec_event_group_t group)
{
    IEC_RETURN_ON_FALSE(ctx, IEC_EVENT_NO_EVENTS, TAG,
                        "Slave interface is not correctly initialized.");
    iecs_controller_iface_t *iecs_controller = IEC_SLAVE_GET_IFACE(ctx);
    IEC_RETURN_ON_FALSE((iecs_controller->check_event && iecs_controller->is_active),
                    IEC_EVENT_NO_EVENTS, TAG,
                    "Slave interface is not correctly configured.");
    iec_event_group_t event = iecs_controller->check_event(ctx, group);
    return event;
}