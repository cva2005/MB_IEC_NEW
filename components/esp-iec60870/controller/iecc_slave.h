/*
 * SPDX-FileCopyrightText: 2016-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "driver/uart.h"            // for uart defines
#include "errno.h"                  // for errno
#include "sys/queue.h"              // for list
#include "esp_log.h"                // for log write
#include "string.h"                 // for strerror()

#ifdef __cplusplus
extern "C" {
#endif

#include "iec_common.h"              // for iec_base_t
#include "esp_iec_slave.h"           // for public type defines

#define IEC_CONTROLLER_NOTIFY_QUEUE_SIZE     (CONFIG_FIEC_CONTROLLER_NOTIFY_QUEUE_SIZE) // Number of messages in parameter notification queue
#define IEC_CONTROLLER_NOTIFY_TIMEOUT        (pdMS_TO_TICKS(CONFIG_FIEC_CONTROLLER_NOTIFY_TIMEOUT)) // notification timeout

/**
 * @brief iec controller handler structure
 */
typedef struct
{
    iec_port_type_t port_type;               /*!< port type */
    iec_communication_info_t comm_opts;      /*!< communication info */
    TaskHandle_t task_handle;                /*!< task handle */
    EventGroupHandle_t event_group_handle;   /*!< controller event group */
    QueueHandle_t notification_queue_handle; /*!< controller notification queue */
} iec_slave_options_t;

typedef iec_event_group_t (*iface_iec_check_event_fp)(void *, iec_event_group_t); /*!< Interface method check_event */

/**
 * @brief Request mode for parameter to use in data dictionary
 */
typedef struct
{
    iec_base_t *iec_base;
    iec_slave_options_t opts;             /*!< iec slave options */
    bool is_active;                       /*!< iec controller interface is active */
    iface_iec_create_fp create;           /*!< Interface factory method */
    iface_iec_method_default_fp delete;   /*!< Interface method delete */
    iface_iec_method_default_fp start;    /*!< Interface method start */
    iface_iec_method_default_fp stop;     /*!< Interface method start */
    iface_iec_check_event_fp check_event; /*!< Interface method check_event */
} iecs_controller_iface_t;

#ifdef __cplusplus
}
#endif