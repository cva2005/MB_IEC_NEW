/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <string.h>
#include "iec_port_tcp_common.h"
#include "iec_port_tcp_slave.h"
#include "iec_port_tcp_driver.h"
#include "iec_port_tcp_utils.h"
#include "iec_transaction.h"
#include "iec_frame.h"
#include "iec_port_common.h"
#include "esp_iec_slave.h"
#include "esp_iec_common.h"
#include "esp_iec_slave.h"
#include "iecc_slave.h"
#include "iecc_tcp_slave.h"
#include "iec_common.h"

typedef struct
{
    iec_port_base_t base;
    // TCP communication properties
    iec_tcp_opts_t tcp_opts;
    iec_uid_info_t addr_info;
    uint8_t ptemp_buf[IEC_TCP_BUFF_MAX_SIZE];
    // The driver object for the slave
    iec_port_driver_t *pdriver;
    transaction_handle_t transaction;
    uint16_t trans_count;
} iecs_tcp_port_t;

/* ----------------------- Static variables & functions ----------------------*/
static const char *TAG = "iec_port.tcp.slave";

static uint64_t iecs_port_tcp_sync_event(void *inst, iec_sync_event_t sync_event);

static esp_err_t iecs_port_tcp_register_handlers(void *ctx)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    esp_err_t ret = ESP_ERR_INVALID_STATE;

    ret = iec_drv_register_handler(pdrv_ctx, IEC_EVENT_READY, iecs_on_ready);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), IEC_EINVAL, TAG,
                       "%x, iec tcp port event registration failed.", (int)IEC_EVENT_READY);
    ret = iec_drv_register_handler(pdrv_ctx, IEC_EVENT_OPEN, iecs_on_open);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), IEC_EINVAL, TAG,
                       "%x, iec tcp port event registration failed.", (int)IEC_EVENT_OPEN);
    ret = iec_drv_register_handler(pdrv_ctx, IEC_EVENT_CONNECT, iecs_on_connect);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), IEC_EINVAL, TAG,
                       "%x, iec tcp port event registration failed.", (int)IEC_EVENT_CONNECT);
    ret = iec_drv_register_handler(pdrv_ctx, IEC_EVENT_ERROR, iecs_on_error);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), IEC_EINVAL, TAG,
                       "%x, iec tcp port event registration failed.", (int)IEC_EVENT_ERROR);
    ret = iec_drv_register_handler(pdrv_ctx, IEC_EVENT_SEND_DATA, iecs_on_send_data);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), IEC_EINVAL, TAG,
                       "%x, iec tcp port event registration failed.", (int)IEC_EVENT_SEND_DATA);
    ret = iec_drv_register_handler(pdrv_ctx, IEC_EVENT_RECV_DATA, iecs_on_recv_data);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), IEC_EINVAL, TAG,
                       "%x, iec tcp port event registration failed.", (int)IEC_EVENT_RECV_DATA);
    ret = iec_drv_register_handler(pdrv_ctx, IEC_EVENT_CLOSE, iecs_on_close);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), IEC_EINVAL, TAG,
                       "%x, iec tcp port event registration failed.", (int)IEC_EVENT_CLOSE);
    ret = iec_drv_register_handler(pdrv_ctx, IEC_EVENT_TIMEOUT, iecs_on_timeout);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), IEC_EINVAL, TAG,
                       "%x, iec tcp port event registration failed.", (int)IEC_EVENT_TIMEOUT);
    return ESP_OK;
}

static esp_err_t iecs_port_tcp_unregister_handlers(void *ctx)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    esp_err_t ret = ESP_ERR_INVALID_STATE;
    ESP_LOGD(TAG, "%p, event handler %p, unregister.", pdrv_ctx, pdrv_ctx->event_handler);

    ret = iec_drv_unregister_handler(pdrv_ctx, IEC_EVENT_READY);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), IEC_EINVAL, TAG,
                       "%x, iec tcp port event registration failed.", (int)IEC_EVENT_READY);
    ret = iec_drv_unregister_handler(pdrv_ctx, IEC_EVENT_OPEN);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), IEC_EINVAL, TAG,
                       "%x, iec tcp port event registration failed.", (int)IEC_EVENT_OPEN);
    ret = iec_drv_unregister_handler(pdrv_ctx, IEC_EVENT_CONNECT);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), IEC_EINVAL, TAG,
                       "%x, iec tcp port event registration failed.", (int)IEC_EVENT_CONNECT);
    ret = iec_drv_unregister_handler(pdrv_ctx, IEC_EVENT_SEND_DATA);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), IEC_EINVAL, TAG,
                       "%x, iec tcp port event registration failed.", (int)IEC_EVENT_SEND_DATA);
    ret = iec_drv_unregister_handler(pdrv_ctx, IEC_EVENT_RECV_DATA);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), IEC_EINVAL, TAG,
                       "%x, iec tcp port event registration failed.", (int)IEC_EVENT_RECV_DATA);
    ret = iec_drv_unregister_handler(pdrv_ctx, IEC_EVENT_CLOSE);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), IEC_EINVAL, TAG,
                       "%x, iec tcp port event registration failed.", (int)IEC_EVENT_CLOSE);
    ret = iec_drv_unregister_handler(pdrv_ctx, IEC_EVENT_TIMEOUT);
    IEC_RETURN_ON_FALSE((ret == ESP_OK), IEC_EINVAL, TAG,
                       "%x, iec tcp port event registration failed.", (int)IEC_EVENT_TIMEOUT);
    return ESP_OK;
}

iec_err_enum_t iecs_port_tcp_create(iec_tcp_opts_t *tcp_opts, iec_port_base_t **port_obj)
{
    IEC_RETURN_ON_FALSE((port_obj && tcp_opts), IEC_EINVAL, TAG, "iec tcp port invalid arguments.");
    iecs_tcp_port_t *ptcp = NULL;
    esp_err_t err = ESP_ERR_INVALID_STATE;
    ptcp = (iecs_tcp_port_t *)calloc(1, sizeof(iecs_tcp_port_t));
    IEC_RETURN_ON_FALSE((ptcp && port_obj), IEC_EILLSTATE, TAG, "iec tcp port creation error.");
    CRITICAL_SECTION_INIT(ptcp->base.lock);
    iec_err_enum_t ret = IEC_EILLSTATE;

    // Copy object descriptor from parent object (is used for logging)
    ptcp->base.descr = ((iec_port_base_t *)*port_obj)->descr;
    ptcp->pdriver = NULL;
    ptcp->transaction = transaction_init();
    IEC_GOTO_ON_FALSE((ptcp->transaction), IEC_EILLSTATE, error,
                     TAG, "iec transaction init failed.");

    ESP_MEM_CHECK(TAG, ptcp->transaction, goto error);

    err = iec_drv_register(&ptcp->pdriver);
    IEC_GOTO_ON_FALSE(((err == ESP_OK) && ptcp->pdriver), IEC_EILLSTATE, error,
                     TAG, "iec tcp port driver registration failed, err = (%x).", (int)err);

    err = iecs_port_tcp_register_handlers(ptcp->pdriver);
    IEC_GOTO_ON_FALSE(((err == ESP_OK) && ptcp->pdriver), IEC_EILLSTATE, error,
                     TAG, "iec tcp port driver registration failed, err = (%x).", (int)err);

    ptcp->pdriver->parent = ptcp; // just for logging purposes
    ptcp->tcp_opts = *tcp_opts;
    ptcp->pdriver->network_iface_ptr = tcp_opts->ip_netif_ptr;
    ptcp->pdriver->iec_proto = tcp_opts->mode;
    //ptcp->pdriver->uid = tcp_opts->uid;
    ptcp->pdriver->is_master = false;
    ptcp->pdriver->event_cbs.iec_sync_event_cb = iecs_port_tcp_sync_event;
    ptcp->pdriver->event_cbs.port_arg = (void *)ptcp;

#ifdef IEC_MDNS_IS_INCLUDED
err = port_start_mdns_service(&ptcp->pdriver->dns_name, false, tcp_opts->uid, ptcp->pdriver->network_iface_ptr);
    IEC_GOTO_ON_FALSE((err == ESP_OK), IEC_EILLSTATE, error, 
                        TAG, "iec tcp port mdns service init failure.");
    ESP_LOGD(TAG, "Start mdns for @%p", ptcp);
#endif
    // ptcp->base.cb.tmr_expired = iecs_port_timer_expired;
    ptcp->base.cb.tx_empty = NULL;
    ptcp->base.cb.byte_rcvd = NULL;
    ptcp->base.arg = (void *)ptcp;
    *port_obj = &(ptcp->base);
    ESP_LOGD(TAG, "created object @%p", ptcp);
    return IEC_ENOERR;

error:
    if (ptcp && ptcp->transaction)
    {
        transaction_destroy(ptcp->transaction);
    }
#ifdef IEC_MDNS_IS_INCLUDED
    port_stop_mdns_service(&ptcp->pdriver->dns_name);
#endif
    if (ptcp && ptcp->pdriver)
    {
        if (ptcp->pdriver->event_handler)
        {
            iecs_port_tcp_unregister_handlers(ptcp->pdriver);
            ptcp->pdriver->event_handler = NULL;
        }
        (void)iec_drv_unregister(ptcp->pdriver);
        CRITICAL_SECTION_CLOSE(ptcp->base.lock);
    }
    free(ptcp);
    return ret;
}

void iecs_port_tcp_delete(iec_port_base_t *inst)
{
    iecs_tcp_port_t *port_obj = __containerof(inst, iecs_tcp_port_t, base);
    if (port_obj && port_obj->transaction)
    {
        transaction_destroy(port_obj->transaction);
    }
#ifdef IEC_MDNS_IS_INCLUDED
    port_stop_mdns_service(&port_obj->pdriver->dns_name);
#endif
    if (port_obj && port_obj->pdriver)
    {
        if (port_obj->pdriver->event_handler)
        {
            iecs_port_tcp_unregister_handlers(port_obj->pdriver);
            port_obj->pdriver->event_handler = NULL;
        }
        (void)iec_drv_unregister(port_obj->pdriver);
        CRITICAL_SECTION_CLOSE(port_obj->base.lock);
    }
    CRITICAL_SECTION_CLOSE(inst->lock);
    free(port_obj);
}

void iecs_port_tcp_enable(iec_port_base_t *inst)
{
    iecs_tcp_port_t *port_obj = __containerof(inst, iecs_tcp_port_t, base);
    (void)iec_drv_start_task(port_obj->pdriver);
    IEC_DRIVER_SEND_EVENT(port_obj->pdriver, IEC_EVENT_READY, UNDEF_FD);
}

void iecs_port_tcp_disable(iec_port_base_t *inst)
{
    iecs_tcp_port_t *port_obj = __containerof(inst, iecs_tcp_port_t, base);
    // Change the state of all slaves to close
    IEC_DRIVER_SEND_EVENT(port_obj->pdriver, IEC_EVENT_CLOSE, UNDEF_FD);
    (void)iec_drv_wait_status_flag(port_obj->pdriver, IEC_FLAG_DISCONNECTED, pdMS_TO_TICKS(IEC_RECONNECT_TIME_MS));
}

bool iecs_port_tcp_recv_data(iec_port_base_t *inst, uint8_t **ppframe, uint16_t *plength)
{
    iecs_tcp_port_t *port_obj = __containerof(inst, iecs_tcp_port_t, base);
    iec_port_driver_t *pdrv_ctx = port_obj->pdriver;
    iec_node_info_t *pnode = NULL;
    bool status = false;
    transaction_item_handle_t item;

    if (plength && ppframe && *ppframe)
    {
        iec_drv_lock(pdrv_ctx);
        item = transaction_get_first(port_obj->transaction);
        if (item && (transaction_item_get_state(item) == ACKNOWLEDGED))
        {
            uint16_t tid = 0;
            int node_id = 0;
            size_t len = 0;
            uint8_t *pbuf = transaction_item_get_data(item, &len, &tid, &node_id);
            pnode = iec_drv_get_node(pdrv_ctx, node_id);
            if (pbuf && pnode && (IEC_GET_NODE_STATE(pnode) >= IEC_SOCK_STATE_CONNECTED))
            {
                memcpy(*ppframe, pbuf, len);
                //*ppframe = pbuf;
                *plength = (uint16_t)len;
                status = true;
                ESP_LOGD(TAG, "%p, " IEC_NODE_FMT(", get packet TID: 0x%04" PRIx16 ", %p."),
                         port_obj, pnode->index, pnode->sock_id,
                         pnode->addr_info.ip_addr_str, (unsigned)pnode->tid_counter, *ppframe);
                if (ESP_OK != transaction_item_set_state(item, CONFIRMED)) {
                    ESP_LOGE(TAG, "transaction queue set state fail.");
                }
            }
        } else {
            // Delete expired frames
            int frame_cnt = transaction_delete_expired(port_obj->transaction,
                                                       iec_port_get_timestamp(),
                                                       (1000 * CONFIG_IEC_MASTER_TIMEOUT_MS_RESPOND));
            if (frame_cnt) {
                ESP_LOGE(TAG, "Deleted %d expired frames.", frame_cnt);
            }
        }
        iec_drv_unlock(pdrv_ctx);
    }
    return status;
}

bool iecs_port_tcp_send_data(iec_port_base_t *inst, uint8_t *pframe, uint16_t length)
{
    iecs_tcp_port_t *port_obj = __containerof(inst, iecs_tcp_port_t, base);

    IEC_RETURN_ON_FALSE((pframe && (length > 0)), false, TAG, "incorrect arguments.");
    bool frame_sent = false;
    iec_port_driver_t *pdrv_ctx = port_obj->pdriver;
    iec_drv_lock(pdrv_ctx);
    int write_length = iec_drv_write(pdrv_ctx, 0, pframe, length);
    if (write_length)
        frame_sent = true;
#if 0
    item = transaction_dequeue(port_obj->transaction, CONFIRMED, NULL);
    if (item) {
        uint16_t msg_id = 0;
        int node_id = 0;
        uint8_t *pbuf = transaction_item_get_data(item, NULL, &msg_id, &node_id);
        if (pbuf && (tid == msg_id)) {
            iec_node_info_t *pnode = iec_drv_get_node(pdrv_ctx, node_id);
            int write_length = iec_drv_write(pdrv_ctx, node_id, pframe, length);
            if (pnode && write_length) {
                frame_sent = true;
                ESP_LOGD(TAG, "%p, node: #%d, socket(#%d)[%s], send packet TID: 0x%04" PRIx16 ":0x%04" PRIx16 ", %p, len: %d, ",
                            pdrv_ctx, pnode->index, pnode->sock_id,
                            pnode->addr_info.node_name_str, (unsigned)tid, (unsigned)msg_id, pframe, length);
            } else {
                ESP_LOGE(TAG, "%p, node: #%d, socket(#%d)[%s], iecs_write fail, TID: 0x%04" PRIx16 ":0x%04" PRIx16 ", %p, len: %d, ",
                            pdrv_ctx, pnode->index, pnode->sock_id,
                            pnode->addr_info.node_name_str, (unsigned)tid, (unsigned)msg_id, pframe, length);
            }
            if (ESP_OK != transaction_item_set_state(item, REPLIED)) {
                ESP_LOGE(TAG, "transaction queue set state fail.");
            }
        }
    } else {
        ESP_LOGE(TAG, "queue can not find the item to send.");
    }
#endif
    iec_drv_unlock(pdrv_ctx);

    if (!frame_sent)
    {
        ESP_LOGE(TAG, "incorrect frame to send.");
    }
    vTaskDelay(TRANSACTION_TICKS);
    return frame_sent;
}

static uint64_t iecs_port_tcp_sync_event(void *inst, iec_sync_event_t sync_event)
{
    switch (sync_event)
    {
        case IEC_SYNC_EVENT_RECV_OK:
            iec_port_timer_disable(inst);
            iec_port_event_set_err_type(inst, _EV_ERROR_INIT);
            iec_port_event_post(inst, _EVENT(_EV_FRAME_RECEIVED));
            break;

        case IEC_SYNC_EVENT_RECV_FAIL:
            iec_port_timer_disable(inst);
            iec_port_event_set_err_type(inst, _EV_ERROR_RECEIVE_DATA);
            iec_port_event_post(inst, _EVENT(_EV_ERROR_PROCESS));
            break;

        case IEC_SYNC_EVENT_SEND_OK:
            iec_port_event_post(inst, _EVENT(_EV_FRAME_SENT));
            break;
        default:
            break;
    }
    return iec_port_get_trans_id(inst);
}

IEC_EVENT_HANDLER(iecs_on_ready)
{
    // The driver is registered
    iec_event_info_t *pevent_info = (iec_event_info_t *)data;
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    iecs_tcp_port_t *port_obj = __containerof(pdrv_ctx->parent, iecs_tcp_port_t, base);
    ESP_LOGD(TAG, "%s  %s: fd: %d", (char *)base, __func__, (int)pevent_info->opt_fd);
    ESP_LOGD(TAG, "addr_table:%s, addr_type:%d, mode:%d, port:%d", *(char **)port_obj->tcp_opts.ip_addr_table,
             (int)port_obj->tcp_opts.addr_type,
             (int)port_obj->tcp_opts.mode,
             (int)port_obj->tcp_opts.port);

    int listen_sock = iec_port_bind_addr(*(char **)port_obj->tcp_opts.ip_addr_table,
                                         port_obj->tcp_opts.addr_type,
                                         port_obj->tcp_opts.mode,
                                         port_obj->tcp_opts.port);
    if (listen_sock < 0)
    {
        iec_drv_check_suspend_shutdown(ctx);
        ESP_LOGE(TAG, "%s, sock: %d, bind error", (char *)base, listen_sock);
        iec_drv_lock(pdrv_ctx);
        if (pdrv_ctx->retry_cnt) pdrv_ctx->retry_cnt--;
        iec_drv_unlock(pdrv_ctx);
        if (pdrv_ctx->retry_cnt) {
            vTaskDelay(TRANSACTION_TICKS);
            IEC_DRIVER_SEND_EVENT(ctx, IEC_EVENT_READY, UNDEF_FD);
        } else {
            IEC_DRIVER_SEND_EVENT(ctx, IEC_EVENT_CLOSE, UNDEF_FD);
            ESP_LOGE(TAG, "%s, stop binding.", (char *)base);
            // iecs_port_tcp_disable(&port_obj->base);
        }
    }
    else
    {
        iec_drv_lock(ctx);
        pdrv_ctx->listen_sock_fd = listen_sock;
        iec_drv_unlock(ctx);
        ESP_LOGD(TAG, "%s  %s: fd: %d, bind is done", (char *)base, __func__, (int)pevent_info->opt_fd);
    }
}

IEC_EVENT_HANDLER(iecs_on_open)
{
    iec_event_info_t *pevent_info = (iec_event_info_t *)data;
    //port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    ESP_LOGD(TAG, "%s  %s: fd: %d", (char *)base, __func__, (int)pevent_info->opt_fd);
}

IEC_EVENT_HANDLER(iecs_on_connect)
{
    iec_event_info_t *pevent_info = (iec_event_info_t *)data;
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    ESP_LOGD(TAG, "%s  %s: fd: %d", (char *)base, __func__, (int)pevent_info->opt_fd);
    iec_node_info_t *pnode = iec_drv_get_node(pdrv_ctx, pevent_info->opt_fd);
    if (!pnode) {
        ESP_LOGD(TAG, "%s %s: fd: %d, is closed.", (char *)base, __func__, (int)pevent_info->opt_fd);
        return;
    }
    tcp_new_connect();
    iec_drv_lock(ctx);
    IEC_SET_NODE_STATE(pnode, IEC_SOCK_STATE_CONNECTED);
    FD_SET(pnode->sock_id, &pdrv_ctx->conn_set);
    iec_drv_unlock(ctx);
}

IEC_EVENT_HANDLER(iecs_on_recv_data)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    iec_event_info_t *pevent_info = (iec_event_info_t *)data;
    ESP_LOGD(TAG, "%s  %s: fd: %d", (char *)base, __func__, (int)pevent_info->opt_fd);
    iec_node_info_t *pnode = iec_drv_get_node(pdrv_ctx, pevent_info->opt_fd);
    if (pnode)
    {
        if (!iec_queue_is_empty(pnode->rx_queue))
        {
            ESP_LOGD(TAG, "%p, node #%d(%d) [%s], receive data ready.", ctx, (int)pevent_info->opt_fd,
                     (int)pnode->sock_id, pnode->addr_info.ip_addr_str);
            uint16_t length = iec_queue_pop(pnode->rx_queue, get_tcp_buff_ptr(), IEC_BUFFER_SIZE, NULL);
            tcp_frame_ready(length);
            ESP_LOGD(TAG, "received packet LEN: %d.", length);
        }
    }
    iec_drv_check_suspend_shutdown(ctx);
}

IEC_EVENT_HANDLER(iecs_on_send_data)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    iec_event_info_t *pevent_info = (iec_event_info_t *)data;
    ESP_LOGD(TAG, "%s  %s: fd: %d", (char *)base, __func__, (int)pevent_info->opt_fd);
    iec_node_info_t *pnode = iec_drv_get_node(pdrv_ctx, pevent_info->opt_fd);
    if (pnode && !iec_queue_is_empty(pnode->tx_queue))
    {
        iec_frame_entry_t frame_entry;
        // pop the frame entry, keep the buffer
        size_t sz = iec_queue_pop(pnode->tx_queue, NULL, IEC_BUFFER_SIZE, &frame_entry);
        if (!sz || (IEC_GET_NODE_STATE(pnode) < IEC_SOCK_STATE_CONNECTED)) {
            ESP_LOGE(TAG, "%p, "IEC_NODE_FMT(", is invalid, drop data."),
                            ctx, (int)pnode->index, (int)pnode->sock_id, pnode->addr_info.ip_addr_str);
            return;
        }
        uint16_t tid = IEC_TCP_IECAP_GET_FIELD(frame_entry.pbuf, IEC_TCP_TID);
        pnode->error = 0;
        int ret = iec_port_write_poll(pnode, frame_entry.pbuf, sz, IEC_TCP_SEND_TIMEOUT_MS);
        if (ret < 0)
        {
            ESP_LOGE(TAG, "%p, " IEC_NODE_FMT(", send data failure, err(errno) = %d(%u)."),
                     ctx, (int)pnode->index, (int)pnode->sock_id,
                     pnode->addr_info.ip_addr_str, (int)ret, (unsigned)errno);
            IEC_DRIVER_SEND_EVENT(ctx, IEC_EVENT_ERROR, pnode->index);
            pnode->error = ret;
        }
        else
        {
            pnode->error = 0;
            ESP_LOGD(TAG, "%p, " IEC_NODE_FMT(", send data successful: TID:0x%04" PRIx16 ":0x%04" PRIx16 ", %d (bytes), errno %d"),
                     ctx, (int)pnode->index, (int)pnode->sock_id,
                     pnode->addr_info.ip_addr_str, pnode->tid_counter, tid, (int)ret, (unsigned)errno);
            ESP_LOG_BUFFER_HEX_LEVEL("SENT", frame_entry.pbuf, ret, ESP_LOG_DEBUG);
        }
        (void)iec_drv_set_status_flag(pdrv_ctx, IEC_FLAG_TRANSACTION_DONE);
        pdrv_ctx->event_cbs.iec_sync_event_cb(pdrv_ctx->event_cbs.port_arg, IEC_SYNC_EVENT_SEND_OK);
        iec_drv_lock(pdrv_ctx);
        free(frame_entry.pbuf);
        pnode->send_time = esp_timer_get_time();
        pnode->send_counter = (pnode->send_counter < (USHRT_MAX - 1)) ? (pnode->send_counter + 1) : 0;
        iec_drv_unlock(pdrv_ctx);
    }
}

IEC_EVENT_HANDLER(iecs_on_error)
{
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    iec_event_info_t *pevent_info = (iec_event_info_t *)data;
    ESP_LOGD(TAG, "%s  %s: fd: %d", (char *)base, __func__, (int)pevent_info->opt_fd);
    iec_node_info_t *pnode = iec_drv_get_node(pdrv_ctx, pevent_info->opt_fd);
    if (!pnode) {
        ESP_LOGD(TAG, "%s %s: fd: %d, is closed.", (char *)base, __func__, (int)pevent_info->opt_fd);
        return;
    }
    iec_port_close_connection(pnode);
    iec_drv_lock(ctx);
    // Check connection and unregister slave
    if ((pnode->sock_id > 0) && (FD_ISSET(pnode->sock_id, &pdrv_ctx->conn_set)))
    {
        FD_CLR(pnode->sock_id, &pdrv_ctx->conn_set);
        if (pdrv_ctx->node_conn_count)
        {
            pdrv_ctx->node_conn_count--;
        }
    }
    if (pnode->index && (FD_ISSET(pnode->index, &pdrv_ctx->open_set))) {
        FD_CLR(pnode->index, &pdrv_ctx->open_set);
    }
    iec_drv_unlock(ctx);
    iec_drv_close(pdrv_ctx, pevent_info->opt_fd);
}

IEC_EVENT_HANDLER(iecs_on_close)
{
    iec_event_info_t *pevent_info = (iec_event_info_t *)data;
    ESP_LOGD(TAG, "%s  %s, fd: %d", (char *)base, __func__, (int)pevent_info->opt_fd);
    iec_port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    // if close all sockets event is received
    if (pevent_info->opt_fd < 0)
    {
        (void)iec_drv_clear_status_flag(pdrv_ctx, IEC_FLAG_DISCONNECTED);
        for (int fd = 0; fd < IEC_MAX_FDS; fd++)
        {
            iec_node_info_t *pnode = iec_drv_get_node(pdrv_ctx, fd);
            if (pnode && (IEC_GET_NODE_STATE(pnode) >= IEC_SOCK_STATE_OPENED)
                      && FD_ISSET(pnode->index, &pdrv_ctx->open_set))
            {
                iec_drv_lock(ctx);
                // Check connection and unregister slave
                if ((pnode->sock_id > 0) && (FD_ISSET(pnode->sock_id, &pdrv_ctx->conn_set)))
                {
                    FD_CLR(pnode->sock_id, &pdrv_ctx->conn_set);
                    if (pdrv_ctx->node_conn_count) {
                        pdrv_ctx->node_conn_count--;
                    }
                }
                FD_CLR(pnode->index, &pdrv_ctx->open_set);
                iec_drv_unlock(ctx);
                // close the socket connection, if active
                (void)iec_port_close_connection(pnode);
                // change slave state immediately to release from select
                IEC_SET_NODE_STATE(pnode, IEC_SOCK_STATE_READY);
            }
        }
        (void)iec_drv_set_status_flag(pdrv_ctx, IEC_FLAG_DISCONNECTED);
        iec_drv_check_suspend_shutdown(ctx);
    }
}

IEC_EVENT_HANDLER(iecs_on_timeout)
{
    // Slave timeout triggered
    iec_event_info_t *pevent_info = (iec_event_info_t *)data;
    //port_driver_t *pdrv_ctx = IEC_GET_DRV_PTR(ctx);
    ESP_LOGD(TAG, "%s  %s: fd: %d", (char *)base, __func__, (int)pevent_info->opt_fd);
    // Todo: add network diagnostic here (ping)? Keep empty for now.
    //iec_drv_check_node_state(pdrv_ctx, UNDEF_FD);
    iec_drv_check_suspend_shutdown(ctx);
}

bool iec_socket_is_connected(void)
{
    iecs_controller_iface_t *iface = (iecs_controller_iface_t *)get_tcp_slave_handle();
    if (iface->iec_base != NULL)
    {
        iecs_tcp_port_t *port_obj = __containerof(iface->iec_base->port_obj, iecs_tcp_port_t, base);
        if (port_obj != NULL)
        {
            iec_port_driver_t *pdrv_ctx = port_obj->pdriver;
            if (pdrv_ctx != NULL)
            {
                iec_node_info_t *pnode = iec_drv_get_node(pdrv_ctx, 0);
                if (pnode != NULL)
                {
                    ESP_LOGD(TAG, "NODE_STATE: %d", IEC_GET_NODE_STATE(pnode));
                    return (IEC_GET_NODE_STATE(pnode) >= IEC_SOCK_STATE_CONNECTED);
                }
            }
        }
    }
    return false;
}