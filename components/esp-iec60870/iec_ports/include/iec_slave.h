/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "iec_types.h"

#ifdef __cplusplus
extern "C" {
#endif

iec_err_enum_t iec_set_slv_id(iec_base_t *inst, uint8_t slv_id, bool is_running, uint8_t const *slv_idstr, uint16_t slv_idstr_len);


#ifdef __cplusplus
}
#endif