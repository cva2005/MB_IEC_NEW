/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "sdkconfig.h" // for KConfig options

#ifdef __cplusplus
extern "C" {
#endif

#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#endif

#include <inttypes.h>

#ifdef ESP_IDF_VERSION

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0))
// Features supported from 4.4
#define IEC_TIMER_SUPPORTS_ISR_DISPATCH_METHOD 1
#endif

#endif

#define IEC_TIMER_USE_ISR_DISPATCH_METHOD        (CONFIG_FIEC_TIMER_USE_ISR_DISPATCH_METHOD)

/*! \brief The option is required for correct UART initialization to place handler into IRAM.
 */
#if CONFIG_UART_ISR_IN_IRAM
#define IEC_PORT_SERIAL_ISR_FLAG                 (ESP_INTR_FLAG_IRAM)
#else
#define IEC_PORT_SERIAL_ISR_FLAG                 (ESP_INTR_FLAG_LOWMED)
#endif

/*! \brief The option represents the serial buffer size for RTU and ASCI.
 */
#define IEC_BUFFER_SIZE                          (260)

/*! \brief The option is required for support of SER over TCP.
 */
#define IEC_TCP_UID_ENABLED                      (CONFIG_FIEC_TCP_UID_ENABLED)

/*! \brief The option defines the queue size for event queue.
 */
#define IEC_EVENT_QUEUE_SIZE                     (CONFIG_FIEC_QUEUE_LENGTH)

/*! \brief Maximum number of Modbus functions codes the protocol stack
 *    should support.
 *
 * The maximum number of supported Modbus functions must be greater than
 * the sum of all enabled functions in this file and custom function
 * handlers. If set to small adding more functions will fail.
 */
#define IEC_FUNC_HANDLERS_MAX                    (16)

/*! \brief Number of bytes which should be allocated for the <em>Report Slave ID
 *    </em>command.
 *
 * This number limits the maximum size of the additional segment in the
 * report slave id function. See eIECSetSlaveID(  ) for more information on
 * how to set this value. It is only used if IEC_FUNC_OTHER_REP_SLAVEID_ENABLED
 * is set to <code>1</code>.
 */
#define IEC_FUNC_OTHER_REP_SLAVEID_BUF           (32)

/*! \brief If the <em>Report Slave ID</em> function should be enabled. */
#define IEC_FUNC_OTHER_REP_SLAVEID_ENABLED       (CONFIG_FIEC_CONTROLLER_SLAVE_ID_SUPPORT)

#define IEC_ADDRESS_MAX (247) /*! Biggest possible slave address. */

#ifdef __cplusplus
}
#endif