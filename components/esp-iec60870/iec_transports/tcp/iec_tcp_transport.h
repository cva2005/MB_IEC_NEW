/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stddef.h>
#include "sdkconfig.h"
#include "iec_common.h"
#include "iec_types.h"
#include "iec_transport_common.h"
#include "iec_port_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------- Defines ------------------------------------------*/

// Common definitions for TCP port
#define IEC_TCP_BUF_SIZE         (256 + 7)
#define IEC_TCP_TIMEOUT_MS       (1000)

typedef enum
{
    IEC_TCP_STATE_INIT,              /*!< Receiver is in initial state. */
    IEC_TCP_STATE_ACTIVE,            /*!< Receiver is in active state. */
    IEC_TCP_STATE_ERROR              /*!< If the frame is invalid. */
} iec_tcp_state_enum_t;

typedef struct _iec_trans_base_t iec_trans_base_t;

iec_err_enum_t iecs_tcp_transp_create(iec_tcp_opts_t *tcp_opts, void **in_out_inst);
bool iecs_tcp_transp_delete(iec_trans_base_t *inst);

#ifdef __cplusplus
}
#endif