/*
 * SPDX-FileCopyrightText: 2016-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
// iecc_serial_slave.c
// Implementation of the Modbus controller serial slave

#include <sys/time.h> // for calculation of time stamp in milliseconds
#include "esp_log.h"  // for log_write

#include "esp_modbus_common.h" // for common defines
#include "esp_modbus_slave.h"  // for public slave interface types
#include "iecc_slave.h"         // for private slave interface types
#include "iecc_serial_slave.h"  // for serial slave implementation definitions

#include "iec_common.h" // for iec object types definition

#include "sdkconfig.h" // for KConfig values

static const char *TAG = "iecc_serial.slave";

// Task function
static void iecc_ser_slave_task(void *param)
{
    iec_slave_options_t *iecs_opts = IEC_SLAVE_GET_OPTS(param);
    iecs_controller_iface_t *iecs_iface = IEC_SLAVE_GET_IFACE(param);

    // Main Modbus stack processing cycle
    for (;;)
    {
        BaseType_t status = xEventGroupWaitBits(iecs_opts->event_group_handle,
                                                (BaseType_t)(IEC_EVENT_STACK_STARTED),
                                                pdFALSE, // do not clear bits
                                                pdFALSE,
                                                portMAX_DELAY);
        // Check if stack started then poll for data
        if (status & IEC_EVENT_STACK_STARTED)
        {
            (void)iecs_iface->iec_base->poll(iecs_iface->iec_base);
        }
        // esp_task_wdt_reset();
    }
}

// Controller start function
static esp_err_t iecc_serial_slave_start(void *ctx)
{
    iec_slave_options_t *iecs_opts = IEC_SLAVE_GET_OPTS(ctx);
    iecs_controller_iface_t *iecs_iface = IEC_SLAVE_GET_IFACE(ctx);
    iec_err_enum_t status = IEC_EIO;

    status = iecs_iface->iec_base->enable(iecs_iface->iec_base);
    IEC_RETURN_ON_FALSE((status == IEC_ENOERR), ESP_ERR_INVALID_STATE, TAG,
                       "iec stack enable fail, returned (0x%x).", (int)status);
    // Set the ieccontroller start flag
    EventBits_t flag = xEventGroupSetBits(iecs_opts->event_group_handle,
                                          (EventBits_t)IEC_EVENT_STACK_STARTED);
    IEC_RETURN_ON_FALSE((flag & IEC_EVENT_STACK_STARTED),
                       ESP_ERR_INVALID_STATE, TAG, "iec stack start event set error.");
    iecs_iface->iec_base->descr.parent = ctx;
    iecs_iface->is_active = true;
    return ESP_OK;
}

// Controller stop function
static esp_err_t iecc_serial_slave_stop(void *ctx)
{
    iec_slave_options_t *iecs_opts = IEC_SLAVE_GET_OPTS(ctx);
    iecs_controller_iface_t *iecs_iface = IEC_SLAVE_GET_IFACE(ctx);
    iec_err_enum_t status = IEC_EIO;
    // Clear the ieccontroller start flag
    EventBits_t flag = xEventGroupClearBits(iecs_opts->event_group_handle,
                                            (EventBits_t)IEC_EVENT_STACK_STARTED);
    IEC_RETURN_ON_FALSE((flag & IEC_EVENT_STACK_STARTED),
                       ESP_ERR_INVALID_STATE, TAG, "iec stack start event set error.");

    status = iecs_iface->iec_base->disable(iecs_iface->iec_base);
    IEC_RETURN_ON_FALSE((status == IEC_ENOERR), ESP_ERR_INVALID_STATE, TAG,
                       "iec stack disable fail, returned (0x%x).", (int)status);
    iecs_iface->iec_base->descr.parent = NULL;
    iecs_iface->is_active = false;
    return ESP_OK;
}

// Blocking function to get event on parameter group change for application task
static iec_event_group_t iecc_serial_slave_check_event(void *ctx, iec_event_group_t group)
{
    iec_slave_options_t *iecs_opts = IEC_SLAVE_GET_OPTS(ctx);
    IEC_SLAVE_ASSERT(iecs_opts->event_group_handle);
    BaseType_t status = xEventGroupWaitBits(iecs_opts->event_group_handle, (BaseType_t)group,
                                            pdTRUE, pdFALSE, portMAX_DELAY);
    return (iec_event_group_t)status;
}

// Controller delete function
static esp_err_t iecc_serial_slave_delete(void *ctx)
{
    iecs_controller_iface_t *iecs_iface = IEC_SLAVE_GET_IFACE(ctx);
    iec_slave_options_t *iecs_opts = IEC_SLAVE_GET_OPTS(ctx);
    iec_err_enum_t iec_error = IEC_ENOERR;

    // Check the stack started bit
    BaseType_t status = xEventGroupWaitBits(iecs_opts->event_group_handle,
                                            (BaseType_t)(IEC_EVENT_STACK_STARTED),
                                            pdFALSE,
                                            pdFALSE,
                                            IEC_CONTROLLER_NOTIFY_TIMEOUT);
    if (iecs_iface->is_active || (status & IEC_EVENT_STACK_STARTED))
    {
        ESP_LOGV(TAG, "iec stack is active, try to disable.");
        if (iecc_serial_slave_stop(ctx) != ESP_OK) {
            ESP_LOGE(TAG, "iec stack stop failure.");
        }
    }

    iecs_iface->is_active = false;
    vTaskDelete(iecs_opts->task_handle);
    vEventGroupDelete(iecs_opts->event_group_handle);
    vQueueDelete(iecs_opts->notification_queue_handle);
    iecs_opts->notification_queue_handle = NULL;
    iecs_opts->event_group_handle = NULL;
    iecs_opts->task_handle = NULL;
    iec_error = iecs_iface->iec_base->delete(iecs_iface->iec_base);
    IEC_RETURN_ON_FALSE((iec_error == IEC_ENOERR), ESP_ERR_INVALID_STATE, TAG,
                       "iec stack close failure returned (0x%x).", (int)iec_error);
    // free the controller will be performed in common slave object
    return ESP_OK;
}

static void iecc_serial_slave_iface_free(void *ctx)
{
    iecs_controller_iface_t *iecs_iface = (iecs_controller_iface_t *)(ctx);
    if (iecs_iface)
    {
        if (iecs_iface->opts.task_handle)
        {
            vTaskDelete(iecs_iface->opts.task_handle);
            iecs_iface->opts.task_handle = NULL;
        }
        if (iecs_iface->opts.event_group_handle)
        {
            vEventGroupDelete(iecs_iface->opts.event_group_handle);
            iecs_iface->opts.event_group_handle = NULL;
        }
        if (iecs_iface->opts.notification_queue_handle)
        {
            vQueueDelete(iecs_iface->opts.notification_queue_handle);
        }
        free(iecs_iface); // free the memory allocated for interface
    }   
}

static esp_err_t iecc_serial_slave_controller_create(void **ctx)
{
    IEC_RETURN_ON_FALSE((ctx), ESP_ERR_INVALID_STATE, TAG,
                       "iec stack init interface fail.");
    esp_err_t ret = ESP_ERR_INVALID_STATE;
    iecs_controller_iface_t *iecs_controller_iface = malloc(sizeof(iecs_controller_iface_t));
    IEC_GOTO_ON_FALSE((iecs_controller_iface), ESP_ERR_NO_MEM, error,
                     TAG, "iec stack memory allocation fail.");

    iec_slave_options_t *iecs_opts = &iecs_controller_iface->opts;
    iecs_opts->port_type = IEC_PORT_SERIAL_SLAVE; // set interface port type

    // Initialization of active context of the Modbus controller
    BaseType_t status = 0;
    // Parameter change notification queue
    iecs_opts->event_group_handle = xEventGroupCreate();
    IEC_GOTO_ON_FALSE((iecs_opts->event_group_handle), ESP_ERR_NO_MEM, error,
                     TAG, "iec event group error.");
    // Create controller task
    status = xTaskCreatePinnedToCore((void *)&iecc_ser_slave_task,
                                     "iecc_ser_slave",
                                     IEC_CONTROLLER_STACK_SIZE,
                                     iecs_controller_iface,
                                     IEC_CONTROLLER_PRIORITY,
                                     &iecs_opts->task_handle,
                                     IEC_PORT_TASK_AFFINITY);
    IEC_GOTO_ON_FALSE((status == pdPASS), ESP_ERR_INVALID_STATE, error, TAG,
                     "iec controller task creation error");
    IEC_SLAVE_ASSERT(iecs_opts->task_handle); // The task is created but handle is incorrect

    // Initialize interface function pointers
    iecs_controller_iface->create = iecc_serial_slave_create;
    iecs_controller_iface->delete = iecc_serial_slave_delete;
    iecs_controller_iface->check_event = iecc_serial_slave_check_event;
    iecs_controller_iface->start = iecc_serial_slave_start;
    iecs_controller_iface->stop = iecc_serial_slave_stop;
    iecs_controller_iface->iec_base = NULL;
    *ctx = iecs_controller_iface;
    return ESP_OK;

error:
    iecc_serial_slave_iface_free((void *)iecs_controller_iface);
    return ret;
}

// Initialization of controller
esp_err_t iecc_serial_slave_create(iec_communication_info_t *config, void **ctx)
{
    iecs_controller_iface_t *iecs_controller_iface = NULL;
    IEC_RETURN_ON_FALSE((ctx && config), ESP_ERR_INVALID_STATE, TAG,
                       "iec stack init interface fail.");
    IEC_RETURN_ON_FALSE((!*ctx), ESP_ERR_INVALID_STATE, TAG, "iec stack is not destroyed?");
    iec_serial_opts_t *pcomm_info = &config->ser_opts;

    // Check communication options
    IEC_RETURN_ON_FALSE((pcomm_info->mode == IEC_SER),
                        ESP_ERR_INVALID_ARG, TAG, "iec incorrect mode = (%u).",
                        (unsigned)pcomm_info->mode);
    IEC_RETURN_ON_FALSE((pcomm_info->port <= UART_NUM_MAX), ESP_ERR_INVALID_ARG, TAG,
                       "iec wrong port to set = (%u).", (unsigned)pcomm_info->port);
    IEC_RETURN_ON_FALSE((pcomm_info->parity <= UART_PARITY_ODD), ESP_ERR_INVALID_ARG, TAG,
                       "iec wrong parity option = (%u).", (unsigned)pcomm_info->parity);

    esp_err_t ret = iecc_serial_slave_controller_create((void *)&iecs_controller_iface);
    IEC_GOTO_ON_FALSE((ret == ESP_OK), ESP_ERR_INVALID_STATE, error, TAG,
                     "iecc create returns (0x%x).", (int)ret);

    iec_slave_options_t *iecs_opts = IEC_SLAVE_GET_OPTS(iecs_controller_iface);
    iecs_opts->port_type = IEC_PORT_SERIAL_SLAVE;
    iecs_opts->comm_opts = *config;
    iec_err_enum_t err = IEC_ENOERR;
    void *pinst = (void *)iecs_controller_iface;

    err = iecs_ser_create(pcomm_info, &pinst);
    IEC_GOTO_ON_FALSE((err == IEC_ENOERR), ESP_ERR_INVALID_STATE, error, TAG,
                      "iecs_ser_create returns (0x%x).", (uint16_t)err);
    iecs_controller_iface->iec_base = (iec_base_t *)pinst;
    iecs_controller_iface->is_active = false;
    *ctx = (void *)iecs_controller_iface;
    return ESP_OK;

error:
    if (iecs_controller_iface) {
        if (iecs_controller_iface->iec_base) {
            iecs_controller_iface->iec_base->delete (iecs_controller_iface->iec_base);
            iecs_controller_iface->iec_base = NULL;
        }
        iecc_serial_slave_iface_free((void *)iecs_controller_iface);
        *ctx = NULL;
    }
    return ret;
}