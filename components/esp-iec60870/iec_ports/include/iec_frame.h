/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "iec_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------- Defines ------------------------------------------*/
#define IEC_PDU_SIZE_MAX             253 /*!< Maximum size of a PDU. */
#define IEC_PDU_SIZE_MIN             1   /*!< Function Code */
#define IEC_PDU_FUNC_OFF             0   /*!< Offset of function code in PDU. */
#define IEC_PDU_DATA_OFF             1   /*!< Offset for response data in PDU. */

#define IEC_SER_PDU_SIZE_MAX         IEC_BUFFER_SIZE /*!< Maximum size of a Modbus frame. */
#define IEC_SER_PDU_SIZE_LRC         1   /*!< Size of LRC field in PDU. */
#define IEC_SER_PDU_ADDR_OFF         0   /*!< Offset of slave address in Ser-PDU. */
#define IEC_SER_PDU_PDU_OFF          1   /*!< Offset of Modbus-PDU in Ser-PDU. */
#define IEC_SER_PDU_SIZE_CRC         2   /*!< Size of CRC field in PDU. */

#define IEC_TCP_TID                  0
#define IEC_TCP_PID                  2
#define IEC_TCP_LEN                  4
#define IEC_TCP_UID                  6
#define IEC_TCP_FUNC                 7

#if IEC_MASTER_TCP_ENABLED
#define IEC_SEND_BUF_PDU_OFF     IEC_TCP_FUNC
#else
#define IEC_SEND_BUF_PDU_OFF     IEC_SER_PDU_PDU_OFF
#endif

#define IEC_TCP_BUFF_MAX_SIZE   IEC_TCP_FUNC + IEC_PDU_SIZE_MAX
#define IEC_TCP_HEAD_SIZE       2
#define IEC_SIZE_FIELD          1
#define IEC_TCP_PSEUDO_ADDRESS   (255)
#define IEC_TCP_PROTOCOL_ID      (0)   /* 0 = Modbus Protocol */

#ifdef __cplusplus
}
#endif
