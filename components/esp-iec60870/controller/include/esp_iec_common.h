/*
 * SPDX-FileCopyrightText: 2016-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "driver/uart.h"                    // for UART types
#include "sdkconfig.h"

#if CONFIG_FIEC_EXT_TYPE_SUPPORT
#include "iec_endianness_utils.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "port_common.h"

#if __has_include("esp_check.h")
#include "esp_check.h"
#include "esp_log.h"

#include <inttypes.h>

#include "iec_port_types.h"

#define IEC_RETURN_ON_FALSE(a, err_code, tag, format, ...) ESP_RETURN_ON_FALSE(a, err_code, tag, format __VA_OPT__(,) __VA_ARGS__)

#else

// if cannot include esp_check then use custom check macro

#define IEC_RETURN_ON_FALSE(a, err_code, tag, format, ...) do {                                         \
        if (!(a)) {                                                                                    \
            ESP_LOGE(tag, "%s(%d): " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);        \
            return err_code;                                                                           \
        }                                                                                              \
} while(0)

#endif

#define IEC_SLAVE_ADDR_PLACEHOLDER           (0xFF)
#define IEC_CONTROLLER_STACK_SIZE            (CONFIG_FIEC_CONTROLLER_STACK_SIZE)  // Stack size for IEC controller
#define IEC_CONTROLLER_PRIORITY              (CONFIG_FIEC_PORT_TASK_PRIO - 1)     // priority of IEC controller task
#define IEC_PORT_TASK_AFFINITY               (CONFIG_FIEC_PORT_TASK_AFFINITY)

// Default port defines
#define IEC_PAR_INFO_TOUT                    (10) // Timeout for get parameter info
#define IEC_PARITY_NONE                      (UART_PARITY_DISABLE)
#define IEC_SECTION(lock)                    CRITICAL_SECTION(lock) {}

// The Macros below handle the endianness while transfer N byte data into buffer
#define _XFER_4_RD(dst, src) { \
    *(uint8_t *)(dst)++ = *(uint8_t *)(src + 1); \
    *(uint8_t *)(dst)++ = *(uint8_t *)(src + 0); \
    *(uint8_t *)(dst)++ = *(uint8_t *)(src + 3); \
    *(uint8_t *)(dst)++ = *(uint8_t *)(src + 2); \
    (src) += 4; \
}

#define _XFER_2_RD(dst, src) { \
    *(uint8_t *)(dst)++ = *(uint8_t *)(src + 1); \
    *(uint8_t *)(dst)++ = *(uint8_t *)(src + 0); \
    (src) += 2; \
}

#define _XFER_4_WR(dst, src) { \
    *(uint8_t *)(dst + 1) = *(uint8_t *)(src)++; \
    *(uint8_t *)(dst + 0) = *(uint8_t *)(src)++; \
    *(uint8_t *)(dst + 3) = *(uint8_t *)(src)++; \
    *(uint8_t *)(dst + 2) = *(uint8_t *)(src)++ ; \
}

#define _XFER_2_WR(dst, src) { \
    *(uint8_t *)(dst + 1) = *(uint8_t *)(src)++; \
    *(uint8_t *)(dst + 0) = *(uint8_t *)(src)++; \
}

/**
 * @brief Types of actual IEC implementation
 */
typedef enum
{
    IEC_PORT_SERIAL_MASTER = 0x00,   /*!< port type serial master. */
    IEC_PORT_SERIAL_SLAVE,           /*!< port type serial slave. */
    IEC_PORT_TCP_MASTER,             /*!< port type TCP master. */
    IEC_PORT_TCP_SLAVE,              /*!< port type TCP slave. */
    IEC_PORT_COUNT,                  /*!< port count. */
    IEC_PORT_INACTIVE = 0xFF
} iec_port_type_t;

/**
 * @brief Event group for parameters notification
 */
typedef enum
{
    IEC_EVENT_NO_EVENTS = 0x00,
    IEC_EVENT_CLOCK_SYNC = BIT0,   /*!< IEC Event for clock synchronization command */
    IEC_EVENT_INTERRG_CMD = BIT1,  /*!< IEC Event for interrogation command */
    IEC_EVENT_CMD_ASDU = BIT2,     /*!< IEC Event for other message types */
    IEC_EVENT_RESET_CU = BIT3,     /*!< IEC Event for reset communication unit message */
    IEC_EVENT_LL_CHANGE = BIT4,    /*!< IEC Event for link layer state changes */
    IEC_EVENT_RAW_MESS = BIT5,     /*!< IEC Event uncomment to log messages */
    IEC_EVENT_STACK_STARTED = BIT7 /*!< IEC Event Stack started */
} iec_event_group_t;

typedef struct _iec_base_t iec_base_t;

/*!
 * \brief TCP type of address for communication.
 */
typedef enum iec_addr_type_enum iec_addr_type_t;

/*!
 * \brief TCP communication options structure.
 */
typedef struct iec_port_tcp_opts iec_tcp_opts_t;

/*!
 * \brief serial communication options structure.
 */
typedef struct iec_port_serial_opts iec_serial_opts_t;

/*!
 * \brief Modbus common communication options structure.
 */
typedef struct iec_port_common_opts iec_common_opts_t;

/**
 * @brief Device communication structure to setup controller
 */
typedef union 
{   
    iec_comm_mode_t mode;            /*!< mode option to check the communication object type*/
    iec_common_opts_t common_opts;   /*!< Common options for communication object. */
    iec_tcp_opts_t tcp_opts;         /*!< tcp options for communication object */
    iec_serial_opts_t ser_opts;      /*!< serial options for communication object */
} iec_communication_info_t;

/**
 * common interface method types
 */
typedef esp_err_t (*iface_iec_create_fp)(iec_communication_info_t*, void **);    /*!< Interface method create */
typedef esp_err_t (*iface_iec_method_default_fp)(void *ctx);                     /*!< Interface method default prototype */

#ifdef __cplusplus
}
#endif

