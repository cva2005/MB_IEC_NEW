/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdatomic.h>
#include <sys/fcntl.h>
#include <sys/param.h>
#include "errno.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_netif.h"

#include "iec_port_common.h"
#include "esp_vfs_eventfd.h"
#include "iec_port_tcp_driver.h"
#include "iec_port_tcp_utils.h"

static const char *TAG = "iec_driver";

static esp_event_loop_handle_t iec_drv_loop_handle = NULL;
static int iec_drv_loop_inst_counter = 0;
static char msg_buffer[100]; // The buffer for event debugging (used for all instances)

static const iec_event_msg_t event_msg_table[] = {
    IEC_EVENT_TBL_IT(IEC_EVENT_READY),
    IEC_EVENT_TBL_IT(IEC_EVENT_OPEN),
    IEC_EVENT_TBL_IT(IEC_EVENT_RESOLVE),
    IEC_EVENT_TBL_IT(IEC_EVENT_CONNECT),
    IEC_EVENT_TBL_IT(IEC_EVENT_SEND_DATA),
    IEC_EVENT_TBL_IT(IEC_EVENT_RECV_DATA),
    IEC_EVENT_TBL_IT(IEC_EVENT_ERROR),
    IEC_EVENT_TBL_IT(IEC_EVENT_CLOSE),
    IEC_EVENT_TBL_IT(IEC_EVENT_TIMEOUT),
};

// The function to print event
const char *iec_driver_event_to_name_r(iec_driver_event_t event)
{
    msg_buffer[0] = 0;
    size_t i;
    for (i = 0; i < sizeof(event_msg_table) / sizeof(event_msg_table[0]); ++i) {
        if (event_msg_table[i].event & event) {
            strlcat(msg_buffer, "|", sizeof(msg_buffer));
            strlcat(msg_buffer, event_msg_table[i].msg, sizeof(msg_buffer));
        }
    }
    return msg_buffer;
}

static esp_err_t init_event_fd(void *ctx)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    if (!iec_drv_loop_inst_counter) {
        esp_vfs_eventfd_config_t config = IEC_EVENTFD_CONFIG();
        esp_err_t err = esp_vfs_eventfd_register(&config);
        if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
            ESP_LOGE(TAG, "eventfd registration fail.");
        }
    }
    pdrv_ctx->event_fd = eventfd(0, 0);
    IEC_RETURN_ON_FALSE((pdrv_ctx->event_fd > 0), ESP_ERR_INVALID_STATE, TAG, "eventfd init error.");
    return (pdrv_ctx->event_fd > 0) ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static esp_err_t close_event_fd(void *ctx)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    if (iec_drv_loop_inst_counter) {
        close(pdrv_ctx->event_fd);
    } else {
        ESP_LOGD(TAG, "close eventfd (%d).", (int)pdrv_ctx->event_fd);
        return esp_vfs_eventfd_unregister();
    }
    return ESP_OK;
}

int32_t iec_write_event(void *ctx, iec_event_info_t *pevent)
{
    IEC_RETURN_ON_FALSE((pevent && ctx), -1, TAG, "wrong arguments.");
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    esp_err_t err = esp_event_post_to(iec_drv_loop_handle,
                                      IEC_EVENT_BASE(ctx), (int32_t)pevent->event_id, pevent,
                                      sizeof(iec_event_info_t), IEC_EVENT_TOUT);
    if ((err != ESP_OK))
    {
        ESP_LOGE(TAG, "%p, event loop send fail, err = %d.", ctx, (int)err);
        return -1;
    }
    // send eventfd to just trigger select
    int32_t ret = write(pdrv_ctx->event_fd, (char *)&pevent->val, sizeof(iec_event_info_t));
    return (ret == sizeof(iec_event_info_t)) ? pevent->event_id : -1;
}

static int32_t read_event(void *ctx, iec_event_info_t *pevent)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    IEC_RETURN_ON_FALSE(pevent, ESP_ERR_INVALID_STATE, TAG, "cannot get event.");
    int ret = read(pdrv_ctx->event_fd, (char *)&pevent->val, sizeof(iec_event_info_t));
    return (ret == sizeof(iec_event_info_t)) ? pevent->event_id : -1;
}

static esp_err_t iec_drv_event_loop_init(void *ctx)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    esp_err_t err = ESP_OK;
    /* Create Event loop without task (will be created separately)*/
    esp_event_loop_args_t loop_args = {
        .queue_size = IEC_EVENT_QUEUE_SZ,
        .task_name = NULL
    };
    if (!iec_drv_loop_handle && !iec_drv_loop_inst_counter) {
        err = esp_event_loop_create(&loop_args, &iec_drv_loop_handle);
        IEC_RETURN_ON_FALSE(((err == ESP_OK) && iec_drv_loop_handle), ESP_ERR_INVALID_STATE, 
                                TAG, "create event loop failed, err=%d.", (int)err);
    }
    pdrv_ctx->event_loop_hdl = iec_drv_loop_handle;
    if (asprintf(&pdrv_ctx->loop_name, "loop:%p", ctx) == -1) {
        abort();
    }
    return err;
}

static esp_err_t iec_drv_event_loop_deinit(void *ctx)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    esp_err_t err = ESP_OK;
    // delete event loop
    IEC_RETURN_ON_FALSE((iec_drv_loop_handle), ESP_ERR_INVALID_STATE, 
                                    TAG, "delete event loop failed.");
    if (iec_drv_loop_inst_counter) {
        ESP_LOGD(TAG, "delete loop inst: %s.", pdrv_ctx->loop_name);
        free(pdrv_ctx->loop_name);
        pdrv_ctx->loop_name = NULL;
        iec_drv_loop_inst_counter--;
    }
    if (!iec_drv_loop_inst_counter) {
        err = esp_event_loop_delete(iec_drv_loop_handle);
        ESP_LOGD(TAG, "delete event loop: %p.", iec_drv_loop_handle);
        iec_drv_loop_handle = NULL;
        IEC_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, 
                                TAG, "delete event loop failed, error=%d.", (int)err);
    }
    return err;
}

esp_err_t iec_drv_register_handler(void *ctx, iec_driver_event_t event, iec_event_handler_fp fp)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    esp_err_t ret = ESP_ERR_INVALID_STATE;

    ESP_LOGD(TAG, "%p, event 0x%x, register.", pdrv_ctx, (int)event);

    ret = esp_event_handler_instance_register_with(iec_drv_loop_handle, IEC_EVENT_BASE(ctx), event,
                                                                fp, ctx, &pdrv_ctx->event_handler);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), ESP_ERR_INVALID_STATE , 
                            TAG, "%p, event handler %p, registration error.", pdrv_ctx, pdrv_ctx->event_handler);
    
    return ESP_OK;
}

esp_err_t iec_drv_unregister_handler(void *ctx, iec_driver_event_t event)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    esp_err_t ret = ESP_ERR_INVALID_STATE;
    ESP_LOGD(TAG, "%p, event handler %p, event 0x%x, unregister.", pdrv_ctx, pdrv_ctx->event_handler, (int)event);

    ret = esp_event_handler_instance_unregister_with(iec_drv_loop_handle,
                                                      IEC_EVENT_BASE(ctx), (int32_t)event, pdrv_ctx->event_handler);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), ESP_ERR_INVALID_STATE , 
                        TAG, "%p, event handler %p, unregister error.", pdrv_ctx, pdrv_ctx->event_handler);

    return ESP_OK;
}

static esp_err_t init_queues(iec_node_info_t *iec_node)
{
    iec_node->rx_queue = iec_queue_create(IEC_RX_QUEUE_MAX_SIZE);
    IEC_RETURN_ON_FALSE(iec_node->rx_queue, ESP_ERR_NO_MEM, TAG, "create rx queue failed");
    iec_node->tx_queue = iec_queue_create(IEC_TX_QUEUE_MAX_SIZE);
    IEC_RETURN_ON_FALSE(iec_node->tx_queue, ESP_ERR_NO_MEM, TAG, "create tx queue failed");
    return ESP_OK;
}

static void delete_queues(iec_node_info_t *piec_node)
{
    if (!iec_queue_is_empty(piec_node->rx_queue))
    {
        iec_queue_flush(piec_node->rx_queue);
    }
    if (!iec_queue_is_empty(piec_node->tx_queue))
    {
        iec_queue_flush(piec_node->tx_queue);
    }
    iec_queue_delete(piec_node->rx_queue);
    iec_queue_delete(piec_node->tx_queue);
    piec_node->rx_queue = NULL;
    piec_node->tx_queue = NULL;
}

inline void iec_drv_lock(void *ctx)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    CRITICAL_SECTION_LOCK(pdrv_ctx->lock);
}

inline void iec_drv_unlock(void *ctx)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    CRITICAL_SECTION_UNLOCK(pdrv_ctx->lock);
}

__attribute__((unused))
iec_sock_state_t iec_drv_get_node_state(void *ctx, int fd)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    iec_node_info_t *pnode = pdrv_ctx->iec_nodes[fd];
    return (pnode) ? atomic_load(&pnode->addr_info.state) : IEC_SOCK_STATE_UNDEF;
}

void iec_drv_check_suspend_shutdown(void *ctx)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);

    if (pdrv_ctx->close_done_sema) {
        iec_status_flags_t status = iec_drv_wait_status_flag(ctx, (IEC_FLAG_SHUTDOWN | IEC_FLAG_SUSPEND), 0);
        ESP_LOGD(TAG, "%p, driver check shutdown (%d)...", ctx, (int)status);
        if (status & IEC_FLAG_SHUTDOWN) {
            xSemaphoreGive(pdrv_ctx->close_done_sema);
            ESP_LOGD(TAG, "%p, driver task shutdown...", ctx);
            vTaskDelete(NULL);
        } else if (status & IEC_FLAG_SUSPEND) {
            xSemaphoreGive(pdrv_ctx->close_done_sema);
            ESP_LOGD(TAG, "%p, driver task is suspended...", ctx);
            vTaskSuspend(NULL);
        }
    }
}

iec_status_flags_t iec_drv_set_status_flag(void *ctx, iec_status_flags_t mask)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    return (iec_status_flags_t)xEventGroupSetBits(pdrv_ctx->status_flags_hdl, (EventBits_t)mask);
}

iec_status_flags_t iec_drv_clear_status_flag(void *ctx, iec_status_flags_t mask)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    return (iec_status_flags_t)xEventGroupClearBits(pdrv_ctx->status_flags_hdl, (EventBits_t)mask);
}

iec_status_flags_t iec_drv_wait_status_flag(void *ctx, iec_status_flags_t mask, TickType_t ticks)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    return (iec_status_flags_t)xEventGroupWaitBits(pdrv_ctx->status_flags_hdl,
                                            (BaseType_t)(mask),
                                            pdFALSE,
                                            pdFALSE,
                                            ticks);
}

int iec_drv_open(void *ctx, iec_uid_info_t addr_info, int flags)
{
    int fd = UNDEF_FD;
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    iec_node_info_t *pnode_info = NULL;
    // Find free fd and initialize
    for (fd = 0; fd < IEC_MAX_FDS; fd++) {
        pnode_info = pdrv_ctx->iec_nodes[fd];
        if (!pnode_info) {
            pnode_info = calloc(1, sizeof(iec_node_info_t));
            if (!pnode_info) {
                goto err;
            }
            ESP_LOGD(TAG, "%p, open vfd: %d, sl_addr: %02x, node: %s:%u",
                        ctx, fd, (int8_t)addr_info.uid,
                        addr_info.ip_addr_str, (unsigned)addr_info.port);
            if (init_queues(pnode_info) != ESP_OK) {
                goto err;
            }
            if (pdrv_ctx->iec_node_open_count > IEC_MAX_FDS) {
                goto err;
            }
            iec_drv_lock(ctx);
            pdrv_ctx->iec_node_open_count++;
            pnode_info->index = fd;
            pnode_info->fd = fd;
            pnode_info->sock_id = addr_info.fd;
            pnode_info->error = -1;
            pnode_info->recv_err = -1;
            pnode_info->addr_info = addr_info;
            //pnode_info->addr_info.ip_addr_str = NULL;
            pnode_info->addr_info.index = fd;
            pnode_info->send_time = esp_timer_get_time();
            pnode_info->recv_time = esp_timer_get_time();
            pnode_info->tid_counter = 0;
            pnode_info->send_counter = 0;
            pnode_info->recv_counter = 0;
            pnode_info->is_blocking = ((flags & O_NONBLOCK) == 0);
            pdrv_ctx->iec_nodes[fd] = pnode_info;
            // mark opened node in the open set
            FD_SET(fd, &pdrv_ctx->open_set);
            iec_drv_unlock(ctx);
            IEC_SET_NODE_STATE(pnode_info, IEC_SOCK_STATE_OPENED);
            IEC_DRIVER_SEND_EVENT(ctx, IEC_EVENT_OPEN, fd);
            return fd;
        }
    }

err:
    free(pnode_info);
    pdrv_ctx->iec_nodes[fd] = NULL;
    iec_drv_unlock(ctx);
    return UNDEF_FD;
}

iec_node_info_t *iec_drv_get_node(void *ctx, int fd)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    return pdrv_ctx->iec_nodes[fd];
}

// writes data into tx queue
ssize_t iec_drv_write(void *ctx, int fd, const void *data, size_t size)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    ssize_t ret = -1;

    if (size == 0) {
        return 0;
    }

    iec_node_info_t *pnode_info = pdrv_ctx->iec_nodes[fd];
    if (!pnode_info) {
        errno = EBADF;
        return 0;
    }

    if (IEC_GET_NODE_STATE(pnode_info) >= IEC_SOCK_STATE_CONNECTED) {
        if (iec_queue_push(pnode_info->tx_queue, (void *)data, size, NULL) == ESP_OK) {
            ret = size;
            // Inform FSM that is new frame data is ready to be send
            IEC_DRIVER_SEND_EVENT(ctx, IEC_EVENT_SEND_DATA, pnode_info->index);
        } else {
            // I/O error
            errno = EIO;
        }
    } else {
        // bad file desc
        errno = EBADF;
    }
    return ret;
}

// reads data from rx queue
ssize_t iec_drv_read(void *ctx, int fd, void *data, size_t size)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    iec_node_info_t *pnode_info = pdrv_ctx->iec_nodes[fd];
    if (!pnode_info) {
        errno = EBADF;
        return 0;
    }

    // fd might be in process of closing (close was already called but preempted)
    if (IEC_GET_NODE_STATE(pnode_info) < IEC_SOCK_STATE_CONNECTED) {
        // bad file desc
        errno = EBADF;
        return -1;
    }

    if (size == 0) {
        return 0;
    }

    ssize_t actual_size = -1;
    if ((actual_size = iec_queue_pop(pnode_info->rx_queue, data, size, NULL)) < 0) {
        errno = EAGAIN;
    }

    return actual_size;
}

int iec_drv_close(void *ctx, int fd)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    iec_node_info_t *pnode_info = pdrv_ctx->iec_nodes[fd]; // get address of configuration

    if (!pnode_info) {
        // not valid opened fd
        errno = EBADF;
        return -1;
    }
    
    // stop socket
    if (IEC_GET_NODE_STATE(pnode_info) != IEC_SOCK_STATE_CLOSED) {
        IEC_SET_NODE_STATE(pnode_info, IEC_SOCK_STATE_CLOSED);
        // Do we need to close connection, if the close event is not run
        iec_port_close_connection((iec_node_info_t *)pnode_info);
    }
    iec_drv_lock(ctx);
    FD_CLR(fd, &pdrv_ctx->open_set);
    delete_queues(pnode_info);
    if (pnode_info->addr_info.node_name_str != pnode_info->addr_info.ip_addr_str) {
        free((void *)pnode_info->addr_info.ip_addr_str); // node ip addr string shall be freed
    }
    free((void *)pnode_info->addr_info.node_name_str);
    pnode_info->addr_info.node_name_str = NULL;
    pnode_info->addr_info.ip_addr_str = NULL;
    free(pnode_info);
    pdrv_ctx->iec_nodes[fd] = NULL;
    pdrv_ctx->iec_node_open_count--;
    iec_drv_unlock(ctx);

    return 0;
}

iec_node_info_t *iec_drv_get_next_node_from_set(void *ctx, int *pfd, fd_set *pfdset)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    if (!pfdset || !pfd) {
        return NULL;
    }
    iec_node_info_t *pnode_info = NULL;
    for (int fd = *pfd; fd < IEC_MAX_FDS; fd++) {
        pnode_info = pdrv_ctx->iec_nodes[fd];
        if (pnode_info && (pnode_info->sock_id > 0)
            && (IEC_GET_NODE_STATE(pnode_info) >= IEC_SOCK_STATE_CONNECTED) 
            && (FD_ISSET(pnode_info->index, pfdset) || (FD_ISSET(pnode_info->sock_id, pfdset)))) {
            *pfd = fd;
            //FD_CLR(pnode_info->sock_id, pfdset);
            return pnode_info;
        }
    }
    return NULL;
}

static int iec_drv_register_fds(void *ctx, fd_set *pfdset)
{
    iec_node_info_t *pnode_info = NULL;
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    // Setup select waiting for eventfd && socket events
    FD_ZERO(pfdset);
    int max_fd = UNDEF_FD;
    // Add to the set all connected slaves
    for (int i = 0; i < IEC_MAX_FDS; i++) {
        pnode_info = pdrv_ctx->iec_nodes[i];
        if (pnode_info && pnode_info->sock_id && (IEC_GET_NODE_STATE(pnode_info) >= IEC_SOCK_STATE_CONNECTED)) {
            IEC_ADD_FD(pnode_info->sock_id, max_fd, pfdset);
        }
    }
    // Add event fd events to the set to handle them in one select
    IEC_ADD_FD(pdrv_ctx->event_fd, max_fd, pfdset);
    // Add listen socket to handle incoming connections (for slave only)
    IEC_ADD_FD(pdrv_ctx->listen_sock_fd, max_fd, pfdset);
    return max_fd;
}

// Wait socket ready event during timeout
static int iec_drv_wait_fd_events(void *ctx, fd_set *pfdset, fd_set *perrset, int time_ms)
{
    fd_set readset = *pfdset;
    int ret = 0;
    struct timeval tv;

    if (!ctx || !pfdset) {
        return -1;
    }

    tv.tv_sec = time_ms / 1000;
    tv.tv_usec = (time_ms - (tv.tv_sec * 1000)) * 1000;

    // fill the readset according to the active fds
    int max_fd = iec_drv_register_fds(ctx, &readset);
    if (perrset) {
        *perrset = readset; // initialize error set if used
    }

    ret = select(max_fd + 1, &readset, NULL, perrset, &tv);
    if (ret == 0) {
        // No respond from node during timeout
        ret = ERR_TIMEOUT;
    } else if (ret < 0) {
        ret = -1;
    } 
    *pfdset = readset;
    return ret;
}

esp_err_t iec_drv_start_task(void *ctx)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    (void)iec_drv_clear_status_flag(ctx, IEC_FLAG_SUSPEND);
    ESP_LOGD(TAG, "%p, resume tcp driver task.", ctx);
    vTaskResume(pdrv_ctx->iec_tcp_task_handle);
    return ESP_OK;
}

esp_err_t iec_drv_stop_task(void *ctx)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    esp_err_t err = ESP_ERR_TIMEOUT;
    if (!pdrv_ctx->close_done_sema) {
        pdrv_ctx->close_done_sema = xSemaphoreCreateBinary();
    }
    (void)iec_drv_set_status_flag(ctx, IEC_FLAG_SUSPEND);
    // Check if we can safely suspend the port task (workaround for issue with deadlock in suspend)
    if (!pdrv_ctx->close_done_sema 
            || !(iec_drv_wait_status_flag(ctx, IEC_FLAG_SUSPEND, 1) & IEC_FLAG_SUSPEND) 
            || (xSemaphoreTake(pdrv_ctx->close_done_sema, pdMS_TO_TICKS(IEC_WAIT_DONE_MS)) != pdTRUE)
            ) {
        ESP_LOGD(TAG, "%p, could not stop driver task during timeout.", ctx);
        vTaskSuspend(pdrv_ctx->iec_tcp_task_handle);
        err = ESP_OK;
    }
    ESP_LOGD(TAG, "%p, stop tcp driver task.", ctx);
    if (pdrv_ctx->close_done_sema) {
        vSemaphoreDelete(pdrv_ctx->close_done_sema);
        pdrv_ctx->close_done_sema = NULL;
    }
    return err;
}

// Todo: remove this later
err_t iec_drv_check_node_state(void *ctx, int fd)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    iec_node_info_t *pnode = NULL;
    err_t err = ERR_ABRT;
    int curr_fd = (fd >= 0) ? fd : 0;

    while(((pnode = iec_drv_get_next_node_from_set(ctx, &curr_fd, &pdrv_ctx->conn_set))
                            && (curr_fd < IEC_MAX_FDS))) {
        if (FD_ISSET(pnode->sock_id, &pdrv_ctx->conn_set)) {
            uint64_t last_read_div_us = (esp_timer_get_time() - pnode->recv_time);
            if (last_read_div_us >= (uint64_t)(IEC_RECONNECT_TIME_MS * 1000)) {
                // ESP_LOGD(TAG, "%p, slave: %d, sock: %d, IP:%s, check connection, time = %" PRId64 ", rcv_time: %" PRId64,
                //             ctx, (int)pnode->index, (int)pnode->sock_id, pnode->addr_info.ip_addr_str,
                //             (esp_timer_get_time() / 1000), pnode->recv_time / 1000);
                err = iec_port_check_alive(pnode, 1);
                if ((err < 0) && (err != ERR_INPROGRESS)) {
                    ESP_LOGE(TAG, "Node #%d [%s], connection error.", pnode->index, pnode->addr_info.ip_addr_str);
                } else {
                    ESP_LOGD(TAG, "Node #%d [%s], connection is ok.", pnode->index, pnode->addr_info.ip_addr_str);
                }
                pnode->recv_time = esp_timer_get_time();
                curr_fd++;
            }
        }
    }
    return err;
}

void iec_drv_tcp_task(void *ctx)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    ESP_LOGD(TAG, "Start of driver task.");
    while (1) {
        fd_set readset, errorset;
        // check all active socket and fd events
        int ret = iec_drv_wait_fd_events(ctx, &readset, &errorset, IEC_SELECT_WAIT_MS);
        if (ret == ERR_TIMEOUT) {
            // timeout occured waiting for the vfds
            // ESP_LOGD(TAG, "%p, task select timeout.", ctx);
            IEC_DRIVER_SEND_EVENT(ctx, IEC_EVENT_TIMEOUT, UNDEF_FD);
            iec_drv_check_suspend_shutdown(ctx);
        } else if (ret == -1) {
            // error occured during waiting for vfds activation
            ESP_LOGD(TAG, "%p, task select error.", ctx);
            iec_drv_check_suspend_shutdown(ctx);
            ESP_LOGD(TAG, "%p, socket error, fdset: %" PRIx64, ctx, *(uint64_t *)&errorset);
        } else {
            // Is the fd event triggered, process the event
            if (pdrv_ctx->event_fd && FD_ISSET(pdrv_ctx->event_fd, &readset)) {
                iec_event_info_t iec_event = {0};
                int32_t event_id = read_event(ctx, &iec_event);
                ESP_LOGD(TAG, "%p, fd event get: 0x%02x:%d, %s",
                         ctx, (int)event_id, (int)iec_event.opt_fd, iec_driver_event_to_name_r(event_id));
                iec_drv_check_suspend_shutdown(ctx);
                // Drive the event loop
                esp_err_t err = esp_event_loop_run(iec_drv_loop_handle, pdMS_TO_TICKS(IEC_TCP_EVENT_LOOP_TICK_MS));
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "%p, event loop run, returns fail: %x", ctx, (int)err);
                }
            } else if (pdrv_ctx->listen_sock_fd && FD_ISSET(pdrv_ctx->listen_sock_fd, &readset)) {
                // If something happened on the listen socket, then it is an incoming connection.
                ESP_LOGD(TAG, "%p, listen_sock is active.", ctx);
                iec_uid_info_t node_info;
                int sock_id = iec_port_accept_connection(pdrv_ctx->listen_sock_fd, &node_info);
                if (sock_id) {
                    int fd = iec_drv_open(pdrv_ctx, node_info, 0);
                    if (fd < 0) {
                        ESP_LOGE(TAG, "%p, unable to open node: %s", pdrv_ctx, node_info.ip_addr_str);
                    } else {
                        IEC_DRIVER_SEND_EVENT(ctx, IEC_EVENT_CONNECT, fd);
                    }
                }
            } else {
                // socket event is ready, process each socket event
                iec_drv_check_suspend_shutdown(ctx);
                int curr_fd = 0;
                iec_node_info_t *pnode_info = NULL;
                ESP_LOGD(TAG, "%p, socket event active: %" PRIx64, ctx, *(uint64_t *)&readset);
                while(((pnode_info = iec_drv_get_next_node_from_set(ctx, &curr_fd, &readset))
                           && (curr_fd < IEC_MAX_FDS))) {
                    if (FD_ISSET(pnode_info->sock_id, &pdrv_ctx->conn_set)) {
                        // The data is ready in the socket, read frame and queue
                        FD_CLR(pnode_info->sock_id, &readset);
                        int ret = iec_port_read_packet(pnode_info);
                        if (ret > 0) {
                            ESP_LOGD(TAG, "%p, "IEC_NODE_FMT(", frame received."), ctx, (int)pnode_info->fd,
                                        (int)pnode_info->sock_id, pnode_info->addr_info.ip_addr_str);
                            iec_drv_lock(ctx);
                            pnode_info->recv_time = esp_timer_get_time();
                            iec_drv_unlock(ctx);
                            IEC_DRIVER_SEND_EVENT(ctx, IEC_EVENT_RECV_DATA, pnode_info->index);
                        } else if (ret == ERR_TIMEOUT) {
                            ESP_LOGD(TAG, "%p, "IEC_NODE_FMT(", frame read timeout or closed connection."), ctx, (int)pnode_info->fd,
                                        (int)pnode_info->sock_id, pnode_info->addr_info.ip_addr_str);
                        } else if (ret == ERR_BUF) {
                            // After retries a response with incorrect TID received, process failure.
                            pdrv_ctx->event_cbs.iec_sync_event_cb(pdrv_ctx->event_cbs.port_arg, IEC_SYNC_EVENT_RECV_FAIL);
                            ESP_LOGI(TAG, "%p, "IEC_NODE_FMT(", frame error."), ctx, (int)pnode_info->fd,
                                        (int)pnode_info->sock_id, pnode_info->addr_info.ip_addr_str);
                        } else {
                            if (ret == ERR_CONN) {
                                ESP_LOGE(TAG, "%p, "IEC_NODE_FMT(", connection lost."), ctx, (int)pnode_info->fd,
                                            (int)pnode_info->sock_id, pnode_info->addr_info.ip_addr_str);
                                IEC_DRIVER_SEND_EVENT(ctx, IEC_EVENT_ERROR, pnode_info->index);
                            } else {
                                ESP_LOGE(TAG, "%p, "IEC_NODE_FMT(", critical error=%d, errno=%u."), ctx, (int)pnode_info->fd,
                                        (int)pnode_info->sock_id, pnode_info->addr_info.ip_addr_str, (int)ret, (unsigned)errno);
                            }
                        }
                    }
                    curr_fd++;
                    iec_drv_check_suspend_shutdown(ctx);
                }
            }
        }
    }
}

esp_err_t iec_drv_register(iec_port_driver_t **ctx)
{
    iec_port_driver_t driver_config = IEC_DRIVER_CONFIG_DEFAULT;
    esp_err_t ret = ESP_ERR_INVALID_STATE;

    iec_port_driver_t *pctx = (iec_port_driver_t *)calloc(1, sizeof(iec_port_driver_t));
    IEC_GOTO_ON_FALSE((pctx), ESP_ERR_NO_MEM, error, TAG, "%p, driver allocation fail.", pctx);
    *pctx = driver_config;

    CRITICAL_SECTION_INIT(pctx->lock);

    // create and initialize modbus driver conetext structure
    pctx->iec_nodes = calloc(IEC_MAX_FDS, sizeof(iec_node_info_t *));
    IEC_GOTO_ON_FALSE((pctx->iec_nodes), ESP_ERR_NO_MEM, error, TAG, "%p, node allocation fail.", pctx);

    for (int i = 0; i < IEC_MAX_FDS; i++) {
        pctx->iec_nodes[i] = NULL;
    }

    ret = init_event_fd((void *)pctx);
    IEC_GOTO_ON_FALSE((ret == ESP_OK), ESP_ERR_INVALID_STATE , error, 
                        TAG, "%p, vfs eventfd init error.", pctx);

    ret = iec_drv_event_loop_init((void *)pctx);
    IEC_GOTO_ON_FALSE((ret == ESP_OK), ESP_ERR_INVALID_STATE , error, 
                        TAG, "%p, event loop init error.", pctx);

    pctx->status_flags_hdl = xEventGroupCreate();
    IEC_GOTO_ON_FALSE((pctx->status_flags_hdl), ESP_ERR_INVALID_STATE, error, 
                        TAG, "%p, iec event group error.", pctx);

    iec_drv_loop_inst_counter++;

    // Create task for packet processing
    BaseType_t state = xTaskCreatePinnedToCore(iec_drv_tcp_task,
                                                "iec_drv_tcp_task",
                                                IEC_TASK_STACK_SZ,
                                                pctx,
                                                IEC_TASK_PRIO,
                                                &pctx->iec_tcp_task_handle,
                                                IEC_PORT_TASK_AFFINITY);
    IEC_GOTO_ON_FALSE((state == pdTRUE), ESP_ERR_INVALID_STATE , error, 
                        TAG, "%p, event task creation error.", pctx);
    (void)iec_drv_stop_task(pctx);

    *ctx = pctx;
    pctx->is_registered = true;
    FD_ZERO(&pctx->open_set);
    FD_ZERO(&pctx->conn_set);
    return ESP_OK;

error:
    if (pctx) {
        if (pctx->iec_tcp_task_handle) {
            vTaskDelete(pctx->iec_tcp_task_handle);
        }
        if (iec_drv_loop_handle) {
            (void)esp_event_loop_delete(iec_drv_loop_handle);
            iec_drv_loop_handle = NULL;
            free(pctx->loop_name);
            pctx->loop_name = NULL;
        }
        if (pctx->event_fd) {
            close(pctx->event_fd);
            (void)esp_vfs_eventfd_unregister();
        }
        if (pctx->close_done_sema) {
            vSemaphoreDelete(pctx->close_done_sema);
            pctx->close_done_sema = NULL;
        }
        free(pctx->iec_nodes);
    }
    free(pctx);
    return ret;
}

esp_err_t iec_drv_unregister(void *ctx)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    ESP_LOGD(TAG, "%p, driver unregister.", pdrv_ctx);
    (void)iec_drv_set_status_flag(ctx, IEC_FLAG_SHUTDOWN);
    pdrv_ctx->close_done_sema = xSemaphoreCreateBinary();

    // if no semaphore (alloc issues) or couldn't acquire it, just delete the task
    if (!pdrv_ctx->close_done_sema 
            || !(iec_drv_wait_status_flag(ctx, IEC_FLAG_SHUTDOWN, 0) & IEC_FLAG_SHUTDOWN) 
            || (xSemaphoreTake(pdrv_ctx->close_done_sema, pdMS_TO_TICKS(IEC_WAIT_DONE_MS)) != pdTRUE)
            ) {
        ESP_LOGD(TAG, "%p, driver tasks couldn't exit within timeout -> abruptly deleting the task.", pdrv_ctx);
        vTaskDelete(pdrv_ctx->iec_tcp_task_handle);
    }

    iec_drv_event_loop_deinit(ctx);
    if (pdrv_ctx->close_done_sema) {
        vSemaphoreDelete(pdrv_ctx->close_done_sema);
        pdrv_ctx->close_done_sema = NULL;
    }

    esp_err_t err = close_event_fd(ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "could not close the eventfd handle, err = %d. Already closed?", err);
    }

    for (int i = 0; i < IEC_MAX_FDS; i++) {
        iec_node_info_t *pnode_info = pdrv_ctx->iec_nodes[i];
        if (pnode_info) {
            ESP_LOGD(TAG, "%p, close node instance #%d(%s).", ctx, i, pnode_info->addr_info.node_name_str);
            iec_drv_close(ctx, i);
        }
    }

    free(pdrv_ctx->iec_nodes); // free the node info address array
    pdrv_ctx->iec_nodes = NULL;

    vEventGroupDelete(pdrv_ctx->status_flags_hdl);

    pdrv_ctx->is_registered = false;
    CRITICAL_SECTION_CLOSE(pdrv_ctx->lock);
    free(pdrv_ctx);

    return ESP_OK;
}

void iec_drv_set_cb(void *ctx, void *conn_cb, void *arg)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    iec_drv_lock(ctx);
    pdrv_ctx->event_cbs.on_conn_done_cb = conn_cb;
    pdrv_ctx->event_cbs.arg = arg;
    iec_drv_unlock(ctx);
}