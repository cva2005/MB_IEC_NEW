/*
 * SPDX-FileCopyrightText: 2016-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// iecc_tcp_slave.c
// Implementation of the Modbus controller TCP slave

#include <sys/time.h>               // for calculation of time stamp in milliseconds
#include "esp_log.h"                // for log_write
#include "sdkconfig.h"              // for KConfig values
#include "esp_iec_common.h"      // for common defines
#include "esp_iec_slave.h"       // for public slave interface types
#include "iecc_slave.h"              // for private slave interface types
#include "iecc_tcp_slave.h"          // for tcp slave iec controller defines
#include "iec_port_tcp_common.h"

#include "iec_common.h"               // for iec types definition

static const char *TAG = "iecc_tcp.slave";

// task function
static void iec_tcp_slave_task(void *param)
{
    iec_slave_options_t *iecs_opts = IEC_SLAVE_GET_OPTS(param);
    iecs_controller_iface_t *iecs_iface = IEC_SLAVE_GET_IFACE(param);

    // Main Modbus stack processing cycle
    for (;;) {
        BaseType_t status = xEventGroupWaitBits(iecs_opts->event_group_handle,
                                                (BaseType_t)(IEC_EVENT_STACK_STARTED),
                                                pdFALSE, // do not clear bits
                                                pdFALSE,
                                                portMAX_DELAY);
        // Check if stack started then poll for data
        if (status & IEC_EVENT_STACK_STARTED) {
            (void)iecs_iface->iec_base->poll(iecs_iface->iec_base);
        }
    }
}

// controller start function
static esp_err_t iecc_tcp_slave_start(void *ctx)
{
    iec_slave_options_t *iecs_opts = IEC_SLAVE_GET_OPTS(ctx);
    iecs_controller_iface_t *iecs_iface = IEC_SLAVE_GET_IFACE(ctx);
    iec_err_enum_t status = IEC_EIO;

    status = iecs_iface->iec_base->enable(iecs_iface->iec_base);
    IEC_RETURN_ON_FALSE((status == IEC_ENOERR), ESP_ERR_INVALID_STATE, TAG,
                        "iec stack enable fail, returned (0x%x).", (uint16_t)status);
    // Set the ieccontroller start flag
    EventBits_t flag = xEventGroupSetBits(iecs_opts->event_group_handle,
                                            (EventBits_t)IEC_EVENT_STACK_STARTED);
    IEC_RETURN_ON_FALSE((flag & IEC_EVENT_STACK_STARTED),
                        ESP_ERR_INVALID_STATE, TAG, "iec stack start event set error.");
    iecs_iface->iec_base->descr.parent = ctx;
    iecs_iface->is_active = true;
    return ESP_OK;
}

// controller stop function
static esp_err_t iecc_tcp_slave_stop(void *ctx)
{
    iec_slave_options_t *iecs_opts = IEC_SLAVE_GET_OPTS(ctx);
    iecs_controller_iface_t *iecs_iface = IEC_SLAVE_GET_IFACE(ctx);
    iec_err_enum_t status = IEC_EIO;

    status = iecs_iface->iec_base->disable(iecs_iface->iec_base);
    IEC_RETURN_ON_FALSE((status == IEC_ENOERR), ESP_ERR_INVALID_STATE, TAG,
                        "iec stack disable fail, returned (0x%x).", (uint16_t)status);
    // Clear the ieccontroller start flag
    EventBits_t flag = xEventGroupClearBits(iecs_opts->event_group_handle,
                                            (EventBits_t)IEC_EVENT_STACK_STARTED);
    IEC_RETURN_ON_FALSE((flag & IEC_EVENT_STACK_STARTED),
                        ESP_ERR_INVALID_STATE, TAG, "iec stack start event set error.");
    iecs_iface->iec_base->descr.parent = NULL;
    iecs_iface->is_active = false;
    return ESP_OK;
}

// Blocking function to get event on parameter group change for application task
static iec_event_group_t iecc_tcp_slave_check_event(void *ctx, iec_event_group_t group)
{
    iec_slave_options_t *iecs_opts = IEC_SLAVE_GET_OPTS(ctx);
    IEC_SLAVE_ASSERT(iecs_opts->event_group_handle);
    BaseType_t status = xEventGroupWaitBits(iecs_opts->event_group_handle, (BaseType_t)group,
                                            pdTRUE , pdFALSE, portMAX_DELAY);
    return (iec_event_group_t)status;
}

// controller delete function
static esp_err_t iecc_tcp_slave_delete(void *ctx)
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
    if (iecs_iface->is_active || (status & IEC_EVENT_STACK_STARTED)) {
        ESP_LOGV(TAG, "iec stack is active, try to disable.");
        IEC_RETURN_ON_FALSE((iecc_tcp_slave_stop(ctx) == ESP_OK), 
                                ESP_ERR_INVALID_STATE, TAG, "iec stack stop failure.");
    }

    iecs_iface->is_active = false;
    vTaskDelete(iecs_opts->task_handle);
    vEventGroupDelete(iecs_opts->event_group_handle);
    vQueueDelete(iecs_opts->notification_queue_handle);
    iec_error = iecs_iface->iec_base->delete(iecs_iface->iec_base);
    IEC_RETURN_ON_FALSE((iec_error == IEC_ENOERR), ESP_ERR_INVALID_STATE, TAG,
                        "iec stack close failure returned (0x%x).", (int)iec_error);
    // free the controller will be performed in common object
    return ESP_OK;
}

esp_err_t iecc_tcp_slave_controller_create(void ** ctx)
{
    IEC_RETURN_ON_FALSE((ctx), ESP_ERR_INVALID_STATE, TAG,
                            "iec stack init interface fail.");
    iecs_controller_iface_t *iecs_controller_iface = *ctx;
    esp_err_t ret = ESP_ERR_INVALID_STATE;
    IEC_RETURN_ON_FALSE((iecs_controller_iface == NULL), ESP_ERR_INVALID_STATE, TAG,
                            "iec stack is not destroyed.");
    
    iecs_controller_iface = malloc(sizeof(iecs_controller_iface_t));
    IEC_GOTO_ON_FALSE((iecs_controller_iface), ESP_ERR_NO_MEM, error, 
                        TAG, "iec stack memory allocation fail.");

    iec_slave_options_t *iecs_opts = &iecs_controller_iface->opts;
    iecs_opts->port_type = IEC_PORT_TCP_SLAVE; // set interface port type

    // Initialization of active context of the Modbus controller
    BaseType_t status = 0;
    // Parameter change notification queue
    iecs_opts->event_group_handle = xEventGroupCreate();
    IEC_GOTO_ON_FALSE((iecs_opts->event_group_handle), ESP_ERR_NO_MEM, error, 
                        TAG, "iec event group error.");
    // Create controller task
    status = xTaskCreatePinnedToCore((void *)&iec_tcp_slave_task,
                                        "iecc_tcp_slave",
                                        IEC_CONTROLLER_STACK_SIZE,
                                        iecs_controller_iface,
                                        IEC_CONTROLLER_PRIORITY,
                                        &iecs_opts->task_handle,
                                        IEC_PORT_TASK_AFFINITY);
    IEC_GOTO_ON_FALSE((status == pdPASS), ESP_ERR_INVALID_STATE, error, TAG, 
                        "iec controller task creation error, xTaskCreate() returns (0x%x).", (uint16_t)status);
    // The task is created but handle is incorrect
    IEC_SLAVE_ASSERT(iecs_opts->task_handle);

    // Initialization of interface pointers
    iecs_controller_iface->create = iecc_tcp_slave_create;
    iecs_controller_iface->delete = iecc_tcp_slave_delete;
    iecs_controller_iface->start = iecc_tcp_slave_start;
    iecs_controller_iface->stop = iecc_tcp_slave_stop;
    iecs_controller_iface->check_event = iecc_tcp_slave_check_event;
    *ctx = iecs_controller_iface;
    return ESP_OK;

error:
    if (iecs_controller_iface) {
        if (iecs_controller_iface->opts.task_handle) {
            vTaskDelete(iecs_controller_iface->opts.task_handle);
            iecs_controller_iface->opts.task_handle = NULL;
        }
        if (iecs_controller_iface->opts.event_group_handle) {
            vEventGroupDelete(iecs_controller_iface->opts.event_group_handle);
            iecs_controller_iface->opts.event_group_handle = NULL;
        }
    }
    free(iecs_controller_iface); // free the memory allocated
    ctx = NULL;
    return ret;
}

// Initialization of controller
esp_err_t iecc_tcp_slave_create(iec_communication_info_t *config, void **ctx)
{
    IEC_RETURN_ON_FALSE((ctx && config), ESP_ERR_INVALID_STATE, TAG,
                            "iec stack init interface fail.");
    iecs_controller_iface_t *iecs_controller_iface = (iecs_controller_iface_t *)*ctx;
    IEC_RETURN_ON_FALSE((!iecs_controller_iface), ESP_ERR_INVALID_STATE, TAG,
                            "iec stack is not destroyed.");
    // Check communication options
    iec_tcp_opts_t tcp_opts = (iec_tcp_opts_t)config->tcp_opts;
    IEC_RETURN_ON_FALSE((tcp_opts.mode == IEC_TCP),
                        ESP_ERR_INVALID_ARG, TAG, "iec transport protocol is incorrect.");
    IEC_RETURN_ON_FALSE((tcp_opts.addr_type == IEC_IPV4),
                        ESP_ERR_INVALID_ARG, TAG, "iec ip address type is incorrect.");
    IEC_RETURN_ON_FALSE((tcp_opts.port), ESP_ERR_INVALID_ARG, TAG, "iec port is not defined.");

    esp_err_t ret = iecc_tcp_slave_controller_create((void *)&iecs_controller_iface);
    IEC_GOTO_ON_FALSE((ret == ESP_OK), ESP_ERR_INVALID_STATE, error, TAG, 
                        "iecc create returns (0x%x).", (uint16_t)ret);

    iec_slave_options_t *iecs_opts = IEC_SLAVE_GET_OPTS(iecs_controller_iface);
    // keep the communication options to be able to restart port driver
    iecs_opts->comm_opts = *config;

    iecs_opts->port_type = IEC_PORT_TCP_SLAVE;
    tcp_opts.mode = IEC_TCP; // Override mode, UDP mode is not supported
    // Keep the response time setting
    if (!tcp_opts.response_tout_ms) {
        tcp_opts.response_tout_ms = CONFIG_FIEC_MASTER_TIMEOUT_MS_RESPOND;
    }
    // Set default values of communication options
    if (!tcp_opts.port) {
        iecs_opts->comm_opts.tcp_opts.port = IEC_TCP_DEFAULT_PORT;
    }

    iecs_opts->comm_opts.tcp_opts = tcp_opts;
    iec_err_enum_t err = IEC_ENOERR;
    void *pinst = (void *)iecs_controller_iface;

    // Initialize stack using ieccontroller parameters
    err = iecs_tcp_create(&tcp_opts, &pinst);
    IEC_GOTO_ON_FALSE((err == IEC_ENOERR), ESP_ERR_INVALID_STATE, error, TAG,
                      "iecs_tcp_create returns (0x%x).", (uint16_t)err);
    iecs_controller_iface->iec_base = (iec_base_t *)pinst;
    iecs_controller_iface->iec_base->descr.is_master = false;
    iecs_controller_iface->is_active = false;
    *ctx = (void *)iecs_controller_iface;
    return ESP_OK;

error:
    if (iecs_controller_iface->iec_base) {
        iecs_controller_iface->iec_base->delete(iecs_controller_iface->iec_base);
        iecs_controller_iface->iec_base = NULL;
    }
    return ret;
}