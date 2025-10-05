/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stddef.h>
#include "iec_config.h"
#include "iec_common.h"
#include "iec_types.h"
#include "iec_frame.h"
#include "iec_transport_common.h"
#include "iec_port_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IEC_RTU_GET_T35_VAL(baudrate) (__extension__(            \
{                                                               \
    uint16_t tmr_35_50us = (baudrate > 19200) ?                 \
                    35 : ((7UL * 220000UL) / (2UL * baudrate)); \
    tmr_35_50us;                                                \
}                                                               \
))

/* ----------------------- Defines ------------------------------------------*/
#define IEC_SER_PDU_SIZE_MIN 4                            /*!< Minimum size of a Ser frame. */
#define IEC_SER_PDU_SIZE_MAX IEC_BUFFER_SIZE              /*!< Maximum size of a Ser frame. */
#define IEC_SER_PDU_SIZE_CRC     2                       /*!< Size of CRC field in PDU. */
#define IEC_SER_PDU_ADDR_OFF     0                       /*!< Offset of slave address in Ser-PDU. */
#define IEC_SER_PDU_PDU_OFF      1                       /*!< Offset of Modbus-PDU in Ser-PDU. */

typedef enum
{
    IEC_SER_STATE_INIT,              /*!< Receiver is in initial state. */
    IEC_SER_STATE_ACTIVE,            /*!< Receiver is in active state. */
    IEC_SER_STATE_ERROR              /*!< If the frame is invalid. */
} iec_ser_state_enum_t;

typedef struct iec_port_serial_opts iec_serial_opts_t;
typedef struct _iec_trans_base_t iec_trans_base_t;

iec_err_enum_t iecs_ser_transp_create(iec_serial_opts_t *ser_opts, void **in_out_inst);
bool iecs_ser_transp_delete(iec_trans_base_t *inst);

#ifdef __cplusplus
}
#endif