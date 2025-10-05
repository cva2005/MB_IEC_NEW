/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <string.h>

#include "iec_config.h"
#include "iec_types.h"
#include "iec_port_types.h"

#ifdef __cplusplus
extern "C" {
#endif


iec_err_enum_t iec_port_ser_create(iec_serial_opts_t *ser_opts, iec_port_base_t **port_obj);
bool iec_port_ser_recv_data(iec_port_base_t *inst, uint8_t **pp_ser_frame, uint16_t *p_ser_length);
bool iec_port_ser_send_data(iec_port_base_t *inst, uint8_t *p_ser_frame, uint16_t ser_length);
void iec_port_ser_enable(iec_port_base_t *inst);
void iec_port_ser_disable(iec_port_base_t *inst);
void iec_port_ser_delete(iec_port_base_t *inst);

#ifdef __cplusplus
}
#endif
