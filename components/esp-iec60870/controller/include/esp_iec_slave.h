/*
 * SPDX-FileCopyrightText: 2016-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

// Public interface header for slave
#include <stdint.h>                 // for standard int types definition
#include <stddef.h>                 // for NULL and std defines
#include "soc/soc.h"                // for BITN definitions
#include "freertos/FreeRTOS.h"      // for task creation and queues access
#include "freertos/event_groups.h"  // for event groups
#include "esp_iec_common.h"      // for common types

#ifdef __cplusplus
extern "C" {
#endif

//#define REG_SIZE(type, nregs) ((type == IEC_PARAM_INPUT) || (type == IEC_PARAM_HOLDING)) ? (nregs >> 1) : (nregs << 3)

#define IEC_SLAVE_ASSERT(con) do { \
        if (!(con)) { ESP_LOGE(TAG, "assert errno:%d, errno_str: !(%s)", errno, strerror(errno)); assert(0 && #con); } \
    } while (0)

#define IEC_SLAVE_GET_IFACE(pctx) (__extension__( \
{ \
    IEC_SLAVE_ASSERT((pctx)); \
    ((iecs_controller_iface_t*)pctx); \
} \
))

#define IEC_SLAVE_GET_OPTS(pctx) (&IEC_SLAVE_GET_IFACE(pctx)->opts)

#define IEC_SLAVE_IS_ACTIVE(pctx) ((bool)(IEC_SLAVE_GET_IFACE(pctx)->is_active))

#define IEC_SLAVE_GET_IFACE_FROM_BASE(pinst) (__extension__( \
{ \
    IEC_SLAVE_ASSERT(pinst); \
    iec_base_t *pbase = (iec_base_t*)pinst; \
    IEC_RETURN_ON_FALSE(pbase->descr.parent, IEC_EILLSTATE, TAG, "Slave interface is not correctly initialized."); \
    ((iecs_controller_iface_t*)pbase->descr.parent); \
} \
))

/**
 * @brief Initialize iec Slave controller and stack for TCP port
 *
 * @param[out] ctx context pointer of the initialized iec interface
 * @param[in] config - pointer to configuration structure for the slave
 *
 * @return
 *     - ESP_OK                 Success
 *     - ESP_ERR_NO_MEM         Parameter error
 *     - ESP_ERR_NOT_SUPPORTED  Port type not supported
 *     - ESP_ERR_INVALID_STATE  Initialization failure
 */
esp_err_t iecc_slave_create_tcp(iec_communication_info_t *config, void **ctx);

/**
 * @brief Initialize iec Slave controller and stack for Serial port
 *
 * @param[out] ctx context pointer of the initialized iec interface
 * @param[in] config - pointer to configuration structure for the slave
 *
 * @return
 *     - ESP_OK                 Success
 *     - ESP_ERR_NO_MEM         Parameter error
 *     - ESP_ERR_NOT_SUPPORTED  Port type not supported
 *     - ESP_ERR_INVALID_STATE  Initialization failure
 */
esp_err_t iecc_slave_create_serial(iec_communication_info_t *config, void **ctx);

/**
 * @brief Initialize iec Slave controller interface handle
 *
 * @param[in] ctx - pointer to slave interface data structure
 */
//void iecc_slave_init_iface(void *ctx);

/**
 * @brief Deletes iec controller and stack engine
 *
 * @param[in] ctx context pointer of the initialized iec interface
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_ERR_INVALID_STATE Parameter error
 */
esp_err_t iecc_slave_delete(void *ctx);

/**
 * @brief Critical section lock function for parameter access
 *
 * @param[in] ctx pointer to slave handle (iec interface)
 * @return
 *     - ESP_OK                 Success
 *     - ESP_ERR_INVALID_STATE  Initialization failure
 */
esp_err_t iecc_slave_lock(void *ctx);

/**
 * @brief Critical section unlock for parameter access
 *
 * @param[in] ctx pointer to slave handle (iec interface)
 * @return
 *     - ESP_OK                 Success
 *     - ESP_ERR_INVALID_STATE  Initialization failure
 */
esp_err_t iecc_slave_unlock(void *ctx);

/**
 * @brief Start of iec communication stack
 *
 * @param[in] ctx context pointer of the initialized iec interface
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_ERR_INVALID_ARG iec stack start error
 */
esp_err_t iecc_slave_start(void *ctx);

/**
 * @brief Stop of iec communication stack
 *
 * @param[in] ctx context pointer of the initialized iec interface
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_ERR_INVALID_ARG iec stack stop error
 */
esp_err_t iecc_slave_stop(void *ctx);

/**
 * @brief Wait for specific event on parameter change.
 *
 * @param[in] ctx context pointer of the initialized iec interface
 * @param group Group event bit mask to wait for change
 *
 * @return
 *     - iec_event_group_t event bits triggered
 */
iec_event_group_t iecc_slave_check_event(void *ctx, iec_event_group_t group);

bool iec_slave_is_connected(void);
bool iec_socket_is_connected(void);
void *get_tcp_slave_handle(void);
uint8_t *get_tcp_buff_ptr(void);
void tcp_frame_ready(uint16_t len);
void tcp_new_connect(void);
bool ser_frame_parse(uint16_t *len, uint8_t **buf);
bool is_iec_ser_start(void);

#ifdef __cplusplus
}
#endif