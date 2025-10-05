/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "stdatomic.h"
#include "iec_config.h"
#include "iec_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IEC_ATTR_WEAK __attribute__ ((weak))

#include "driver/uart.h"

__attribute__((__packed__))
struct iec_port_serial_opts {
    iec_comm_mode_t mode;           /*!< iec communication mode */
    uart_port_t port;               /*!< iec communication port (UART) number */
    uint32_t response_tout_ms;      /*!< iec slave response timeout */
    uint64_t test_tout_us;          /*!< iec test timeout (reserved) */
    uint32_t baudrate;              /*!< iec baudrate */
    uart_word_length_t data_bits;   /*!< iec number of data bits */
    uart_stop_bits_t stop_bits;     /*!< iec number of stop bits */
    uart_parity_t parity;           /*!< iec UART parity settings */
};

typedef struct iec_port_serial_opts iec_serial_opts_t;

typedef enum iec_addr_type_enum
{
    IEC_NOIP = 0,
    IEC_IPV4 = 1, /*!< TCP IPV4 addressing */
    IEC_IPV6 = 2  /*!< TCP IPV6 addressing */
} iec_addr_type_t;

__attribute__((__packed__))
struct iec_port_common_opts {
    iec_comm_mode_t mode;           /*!< communication mode */
    uint16_t port;                  /*!< communication port (UART) number */
    uint32_t response_tout_ms;      /*!< slave response timeout */
    uint64_t test_tout_us;          /*!< test timeout (reserved) */
};

__attribute__((__packed__))
struct iec_port_tcp_opts {
    iec_comm_mode_t mode;           /*!< communication mode */
    uint16_t port;                  /*!< communication port (UART) number */
    uint32_t response_tout_ms;      /*!< slave response timeout */
    uint64_t test_tout_us;          /*!< test timeout (reserved) */
    iec_addr_type_t addr_type;       /*!< address type */
    void *ip_addr_table;            /*!< address or table for connection */
    void *ip_netif_ptr;             /*!< network interface */
    char *dns_name;                 /*!< node DNS name */
    bool start_disconnected;        /*!< (Master only option) do not wait for connection to all nodes before polling */
};

typedef struct iec_port_tcp_opts iec_tcp_opts_t;

// The common object descriptor struture (common for iec, transport, port objects)
struct _iec_obj_descr { 
    char *parent_name;              /*!< Name of the parent (base) object */
    char *obj_name;                 /*!< Name of the object */
    void *parent;                   /*!< Pointer to the parent (base) object */
    uint32_t inst_index;            /*!< The consicutive index of the object instance */
    bool is_master;                 /*!< The current object is master or slave (false) */
};

typedef struct _iec_obj_descr iec_obj_descr_t;

typedef enum _iec_sock_state {
    IEC_SOCK_STATE_UNDEF = 0x0000,   /*!< Default init state */
    IEC_SOCK_STATE_CLOSED,           /*!< Node is closed */
    IEC_SOCK_STATE_READY,            /*!< Node is ready for communication */
    IEC_SOCK_STATE_OPENED,           /*!< Node is opened */
    IEC_SOCK_STATE_RESOLVED,         /*!< Node address is resolved */
    IEC_SOCK_STATE_CONNECTING,       /*!< Node connection is in progress */
    IEC_SOCK_STATE_CONNECTED,        /*!< Node is connected */
    IEC_SOCK_STATE_ACCEPTED          /*!< Slave node accepted the connection */
} iec_sock_state_t;

typedef struct _iec_uid_info {
    uint16_t index;                 /*!< index of the address info */
    int fd;                         /*!< node global FD for VFS (reserved) */
    char *node_name_str;            /*!< node name string (host name of node to resolve) */
    char *ip_addr_str;              /*!< represents the IP address of the node */
    iec_addr_type_t addr_type;      /*!< type of IP address */
    uint16_t uid;                   /*!< node unit ID (UID) field for IECAP frame  */
    uint16_t port;                  /*!< node port number */
    iec_comm_mode_t proto;          /*!< protocol type */
    _Atomic iec_sock_state_t state; /*!< node state */
    void *inst;                     /*!< pointer to linked instance */
} iec_uid_info_t;

#ifdef __cplusplus
}
#endif