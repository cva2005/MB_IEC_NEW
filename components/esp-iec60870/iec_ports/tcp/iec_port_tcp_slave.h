#pragma once

/* ----------------------- Platform includes --------------------------------*/
#include "esp_log.h"

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_timer.h"
#include "sys/time.h"
#include "esp_netif.h"

#include "iec_common.h"
#include "iec_port_tcp_driver.h"
#include "sys/queue.h"

#define TRANSACTION_TICKS pdMS_TO_TICKS(50)

/**
 * @brief Modbus slave addr list item for the master
 */
typedef struct iec_data_entry_s {
    int node_id;
    uint64_t token;
    iec_node_info_t *pnode;
    iec_frame_entry_t frame;
    bool pending;
    STAILQ_ENTRY(iec_data_entry_s) entries;
} iec_data_item_t;


IEC_EVENT_HANDLER(iecs_on_ready);
IEC_EVENT_HANDLER(iecs_on_open);
IEC_EVENT_HANDLER(iecs_on_resolve);
IEC_EVENT_HANDLER(iecs_on_connect);
IEC_EVENT_HANDLER(iecs_on_send_data);
IEC_EVENT_HANDLER(iecs_on_recv_data);
IEC_EVENT_HANDLER(iecs_on_error);
IEC_EVENT_HANDLER(iecs_on_close);
IEC_EVENT_HANDLER(iecs_on_timeout);