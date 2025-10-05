/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <string.h>
#include "iec_config.h"
#include "iec_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IEC_TCP_PORT_MAX_CONN            (CONFIG_FIEC_TCP_PORT_MAX_CONN)
#define IEC_TCP_DEFAULT_PORT             (CONFIG_FIEC_TCP_PORT_DEFAULT)
#define IEC_FRAME_QUEUE_SZ               (20)
#define IEC_TCP_CONNECTION_TIMEOUT_MS    (20)        // connection timeout in mS
#define IEC_TCP_RECONNECT_TIMEOUT        (5000000)   // reconnection timeout in uS

#define IEC_EVENT_SEND_RCV_TOUT_MS       (500)

#define IEC_TCP_IECAP_GET_FIELD(buffer, field) ((uint16_t)((buffer[field] << 8U) | buffer[field + 1]))
#define IEC_TCP_IECAP_SET_FIELD(buffer, field, val) { \
    buffer[(field)] = (uint8_t)((val) >> 8U);       \
    buffer[(field) + 1] = (uint8_t)((val) & 0xFF);  \
}

#define IEC_NODE_FMT(fmt) "node #%d, socket(#%d)(%s)" fmt

iec_err_enum_t iecs_port_tcp_create(iec_tcp_opts_t *tcp_opts, iec_port_base_t **port_obj);
void iecs_port_tcp_delete(iec_port_base_t *inst);
void iecs_port_tcp_enable(iec_port_base_t *inst);
void iecs_port_tcp_disable(iec_port_base_t *inst);
bool iecs_port_tcp_send_data(iec_port_base_t *inst, uint8_t *pframe, uint16_t length);
bool iecs_port_tcp_recv_data(iec_port_base_t *inst, uint8_t **ppframe, uint16_t *plength);

#ifdef __cplusplus
}
#endif