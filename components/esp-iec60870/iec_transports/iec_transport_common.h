/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "iec_types.h"
#include "iec_port_common.h"
#include "iec_port_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _iec_trans_base_t iec_trans_base_t;  /*!< Type of moddus transport object */
//typedef struct _obj_descr obj_descr_t;

typedef void (*iec_frm_start_fp)(iec_trans_base_t *transport);
typedef void (*iec_frm_stop_fp)(iec_trans_base_t *transport);
typedef iec_err_enum_t (*iec_frm_rcv_fp)(iec_trans_base_t *transport, uint8_t **frame_ptr_buf, uint16_t *len_buf);
typedef iec_err_enum_t (*iec_frm_snd_fp)(iec_trans_base_t *transport, const uint8_t *frame_ptr, uint16_t len);
typedef void (*iec_get_rx_frm_fp) (iec_trans_base_t *transport, uint8_t **frame_ptr_buf);
typedef void (*iec_get_tx_frm_fp) (iec_trans_base_t *transport, uint8_t **frame_ptr_buf);
typedef bool (*iec_get_fp)(iec_trans_base_t *inst);

struct _iec_trans_base_t
{
    iec_obj_descr_t descr;

    _lock_t lock;
    iec_port_base_t *port_obj;

    iec_frm_start_fp frm_start;
    iec_frm_stop_fp frm_stop;
    iec_get_fp frm_delete;
    iec_frm_snd_fp frm_send;
    iec_frm_rcv_fp frm_rcv;
    iec_get_rx_frm_fp get_rx_frm;
    iec_get_rx_frm_fp get_tx_frm;
    iec_get_fp frm_is_bcast;
}; //!< Transport methods

#ifdef __cplusplus
}
#endif