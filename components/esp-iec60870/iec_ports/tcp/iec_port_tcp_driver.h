/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

//#include <sys/queue.h>
#include <stdatomic.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_event.h"          // for esp event loop

#if __has_include("mdns.h")
#include "mdns.h"
#endif

#include "iec_config.h"
#include "iec_common.h"

#include "iec_port_tcp_utils.h"
#include "iec_port_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IEC_PORT_DEFAULT         (CONFIG_FIEC_TCP_PORT_DEFAULT)
#define UNDEF_FD                (-1)
#define IEC_EVENT_TOUT           (300 / portTICK_PERIOD_MS)
#define IEC_CONN_TICK_TIMEOUT    (10 / portTICK_PERIOD_MS)

typedef void (*iec_event_handler_fp)(void *ctx, esp_event_base_t base, int32_t id, void *data);
#define IEC_EVENT_HANDLER(handler_name) void (handler_name)(void *ctx, esp_event_base_t base, int32_t id, void *data)

#define IEC_TASK_STACK_SZ            (CONFIG_FIEC_PORT_TASK_STACK_SIZE)
#define IEC_TASK_PRIO                (CONFIG_FIEC_PORT_TASK_PRIO)
#define IEC_PORT_TASK_AFFINITY       (CONFIG_FIEC_PORT_TASK_AFFINITY)

#define IEC_MAX_FDS                  (IEC_TCP_PORT_MAX_CONN)
#define IEC_RETRY_MAX                (2)
#define IEC_RECONNECT_TIME_MS        (1000)
#define IEC_RX_QUEUE_MAX_SIZE        (CONFIG_FIEC_QUEUE_LENGTH)
#define IEC_TX_QUEUE_MAX_SIZE        (CONFIG_FIEC_QUEUE_LENGTH)
#define IEC_EVENT_QUEUE_SZ           (CONFIG_FIEC_QUEUE_LENGTH * IEC_TCP_PORT_MAX_CONN)

#define IEC_WAIT_DONE_MS             (5000)
#define IEC_SELECT_WAIT_MS           (200)
#define IEC_TCP_SEND_TIMEOUT_MS      (500)
#define IEC_TCP_EVENT_LOOP_TICK_MS   (50)

#define IEC_DRIVER_CONFIG_DEFAULT {              \
    .spin_lock = portMUX_INITIALIZER_UNLOCKED,  \
    .listen_sock_fd = UNDEF_FD,                 \
    .retry_cnt = IEC_RETRY_MAX,                  \
    .iec_tcp_task_handle = NULL,                 \
    .iec_node_open_count = 0,                    \
    .curr_node_index = 0,                       \
    .iec_proto = IEC_TCP,                         \
    .network_iface_ptr = NULL,                  \
    .dns_name = NULL,                           \
    .iec_nodes = NULL,                           \
    .iec_node_curr = NULL,                       \
    .close_done_sema = NULL,                    \
    .max_conn_sd = UNDEF_FD,                    \
    .node_conn_count = 0,                       \
    .event_fd = UNDEF_FD,                       \
}

#define IEC_EVENTFD_CONFIG() (esp_vfs_eventfd_config_t) {    \
      .max_fds = IEC_TCP_PORT_MAX_CONN                       \
};

typedef struct _iec_port_driver iec_port_driver_t;

#define IEC_CHECK_FD_RANGE(fd) ((fd < IEC_TCP_PORT_MAX_CONN) && (fd >= 0))

#define IEC_GET_DRV_PTR(ctx) (__extension__( \
{                                           \
    assert(ctx);                            \
    ((iec_port_driver_t *)ctx);                 \
}                                           \
))

#define IEC_EVENT_TBL_IT(event)    {event, #event}

#define IEC_EVENT_BASE(context) (__extension__(                                      \
{                                                                                   \
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(context);                              \
    (pdrv_ctx->loop_name) ? (esp_event_base_t)(pdrv_ctx->loop_name) : "UNK_BASE";   \
}                                                                                   \
))

#define IEC_ADD_FD(fd, max_fd, pfdset) do {      \
    if (fd) {                                   \
        (max_fd = (fd > max_fd) ? fd : max_fd); \
        FD_SET(fd, pfdset);                     \
    }                                           \
} while(0)


// Macro for atomic operations
#define IEC_ATOMIC_LOAD(ctx, addr) (__extension__(   \
{                                                   \
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);  \
    (CRITICAL_LOAD(pdrv_ctx->lock, addr));          \
}                                                   \
))

#define IEC_ATOMIC_STORE(ctx, addr, val) (__extension__( \
{                                                       \
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);      \
    CRITICAL_STORE(pdrv_ctx->lock, addr, val);          \
}                                                       \
))

// Post event to event loop and unblocks the select through the eventfd to handle the event loop run,
// So, the eventfd value keeps last event and its fd.
#define IEC_DRIVER_SEND_EVENT(ctx, event, fd) (__extension__(                               \
{                                                                                       \
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);                                      \
    iec_event_info_t (event_info##__FUNCTION__##__LINE__);                               \
    (event_info##__FUNCTION__##__LINE__).event_id = (int32_t)event;                     \
    (event_info##__FUNCTION__##__LINE__).opt_fd = fd;                                   \
    ((iec_write_event((void *)pdrv_ctx, &(event_info##__FUNCTION__##__LINE__)) > 0)         \
                    ? ((event_info##__FUNCTION__##__LINE__)).event_id : UNDEF_FD);      \
}                                                                                       \
))

#define IEC_GET_NODE_STATE(pnode) (atomic_load(&((iec_node_info_t *)pnode)->addr_info.state))

#define IEC_SET_NODE_STATE(pnode, node_state) do {                               \
    atomic_store(&(((iec_node_info_t *)pnode)->addr_info.state), node_state);    \
} while(0)

typedef enum _iec_driver_event {
    IEC_EVENT_READY = 0x0001,
    IEC_EVENT_OPEN = 0x0002,
    IEC_EVENT_RESOLVE = 0x0004,
    IEC_EVENT_CONNECT = 0x0008,
    IEC_EVENT_SEND_DATA = 0x0010,
    IEC_EVENT_RECV_DATA = 0x0020,
    IEC_EVENT_ERROR = 0x0040,
    IEC_EVENT_CLOSE = 0x0080,
    IEC_EVENT_TIMEOUT = 0x0100
} iec_driver_event_t;

typedef struct {
    iec_driver_event_t event;
    const char *msg;
} iec_event_msg_t;

typedef union {
    struct {
        int32_t event_id;               /*!< an event */
        int32_t opt_fd;                 /*!< fd option for an event */
    };
    uint64_t val;
} iec_event_info_t;

typedef struct _iec_node_info {
    int index;                          /*!< slave information index */
    int fd;                             /*!< slave global file descriptor */
    int sock_id;                        /*!< socket ID of slave */
    iec_uid_info_t addr_info;            /*!< slave address info structure*/
    int error;                          /*!< socket error */
    int recv_err;                       /*!< socket receive error */
    QueueHandle_t rx_queue;             /*!< receive response queue */
    QueueHandle_t tx_queue;             /*!< send request queue */
    int64_t send_time;                  /*!< send request time stamp */
    int64_t recv_time;                  /*!< receive response time stamp */
    uint16_t tid_counter;               /*!< transaction identifier (TID) for slave */
    uint16_t send_counter;              /*!< number of packets sent to slave during one session */
    uint16_t recv_counter;              /*!< number of packets received from slave during one session */
    bool is_blocking;                   /*!< slave blocking bit state saved */
} iec_node_info_t;

typedef enum _iec_sync_event {
    IEC_SYNC_EVENT_RECV_OK = 0x0001,
    IEC_SYNC_EVENT_RECV_FAIL = 0x0002,
    IEC_SYNC_EVENT_SEND_OK = 0x0003,
    IEC_SYNC_EVENT_TOUT
} iec_sync_event_t;

typedef enum _iec_status_flags {
    IEC_FLAG_BLANK = 0x0000,
    IEC_FLAG_TRANSACTION_DONE = 0x0001,
    IEC_FLAG_DISCONNECTED = 0x0002,
    IEC_FLAG_CONNECTED = 0x0004,
    IEC_FLAG_SUSPEND = 0x0008,
    IEC_FLAG_SHUTDOWN = 0x0010
} iec_status_flags_t;

typedef struct _iec_driver_event_cbs {
    void (*on_conn_done_cb)(void *);
    void *arg;
    uint64_t (*iec_sync_event_cb)(void *, iec_sync_event_t);
    void *port_arg;
} iec_driver_event_cb_t;

/**
 * @brief iec driver context parameters
 *
 */
typedef struct _iec_port_driver {
    void *parent;                               /*!< parent object pointer */
    char *dns_name;                             /*!< DNS name of the object */
    portMUX_TYPE spin_lock;                     /*!< spin lock */
    _lock_t lock;                               /*!< semaphore mutex */
    bool is_registered;                         /*!< driver is active flag */
    int listen_sock_fd;                         /*!< listen socket fd */
    int retry_cnt;                              /*!< retry counter for events */
    iec_comm_mode_t iec_proto;                    /*!< current node protocol type */
    uint16_t port;                              /*!< current node port number */
    uint8_t uid;                                /*!< unit identifier of the node */
    bool is_master;                             /*!< identify the type of instance (master, slave) */
    void *network_iface_ptr;                    /*!< netif interface pointer */
    iec_node_info_t **iec_nodes;                  /*!< information structures for each associated node */
    uint16_t iec_node_open_count;                /*!< count of associated nodes */
    uint16_t node_conn_count;                   /*!< number of associated nodes */
    iec_node_info_t *iec_node_curr;               /*!< current slave information */
    uint16_t curr_node_index;                   /*!< current processing slave index */
    fd_set open_set;                            /*!< file descriptor set for opened nodes */
    fd_set conn_set;                            /*!< file descriptor set for associated nodes */
    int max_conn_sd;                            /*!< max file descriptor for associated nodes */
    int event_fd;                               /*!< eventfd descriptor for modbus event tracking */
    SemaphoreHandle_t close_done_sema;          /*!< close and done semaphore */
    EventGroupHandle_t status_flags_hdl;        /*!< status bits to control nodes states */
    TaskHandle_t iec_tcp_task_handle;            /*!< TCP/UDP handling task handle */
    esp_event_loop_handle_t event_loop_hdl;     /*!< event loop handle */
    esp_event_handler_instance_t event_handler; /*!< event handler instance */
    char *loop_name;                            /*!< name for event loop used as base */
    iec_driver_event_cb_t event_cbs;
    //LIST_HEAD(iec_uid_info_, iec_uid_entry_s) node_list; /*!< node address information list */
    //uint16_t node_list_count;
} iec_port_driver_t;

/**
 * @brief Register driver
 *
 * This function must be called prior usage of Interface
 *
 * @param ctx - pointer to pointer of driver interface structure to be created.
 * @param config iec virtual filesystem driver configuration. Default base path /dev/net/iec/tcp is used when this paramenter is NULL.
 * @return esp_err_t
 *          - ESP_OK on success
 */
esp_err_t iec_drv_register(iec_port_driver_t **config);

/**
 * @brief Unregister iec driver
 *
 * @param ctx - pointer to driver interface structure
 * @return esp_err_t
 *          - ESP_OK on success
 */
esp_err_t iec_drv_unregister(void *ctx);

/**
 * @brief Start task of iec driver
 *
 * @param ctx - pointer to driver interface structure
 * @return esp_err_t
 *          - ESP_OK on success
 */
esp_err_t iec_drv_start_task(void *ctx);

/**
 * @brief Unregister iec driver
 *
 * @param ctx - pointer to driver interface structure
 * @return esp_err_t
 *          - ESP_OK on success
 */
esp_err_t iec_drv_stop_task(void *ctx);

iec_node_info_t *iec_drv_get_node(void *ctx, int fd);

iec_sock_state_t iec_drv_get_node_state(void *ctx, int fd);

int iec_drv_open(void *ctx, iec_uid_info_t addr_info, int flags);

ssize_t iec_drv_write(void *ctx, int fd, const void *data, size_t size);

ssize_t iec_drv_read(void *ctx, int fd, void *data, size_t size);

int iec_drv_close(void *ctx, int fd);

int32_t iec_write_event(void *ctx, iec_event_info_t *pevent);

const char *iec_driver_event_to_name_r(iec_driver_event_t event);

void iec_drv_set_cb(void *ctx, void *conn_cb, void *arg);

iec_status_flags_t iec_drv_wait_status_flag(void *ctx, iec_status_flags_t mask, TickType_t ticks);

esp_err_t iec_drv_register_handler(void *ctx, iec_driver_event_t event, iec_event_handler_fp fp);

esp_err_t iec_drv_unregister_handler(void *ctx, iec_driver_event_t event);

void iec_drv_check_suspend_shutdown(void *ctx);

void iec_drv_lock(void *ctx);

void iec_drv_unlock(void *ctx);

iec_node_info_t *iec_drv_get_next_node_from_set(void *ctx, int *pfd, fd_set *pfdset);

iec_status_flags_t iec_drv_set_status_flag(void *ctx, iec_status_flags_t mask);

iec_status_flags_t iec_drv_clear_status_flag(void *ctx, iec_status_flags_t mask);

err_t iec_drv_check_node_state(void *ctx, int fd);

#ifdef __cplusplus
}
#endif
