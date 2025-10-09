/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "iec_common.h"
#include "iec_slave.h"
#include "iec_frame.h"
#include "iec_port_common.h"
#include "iec_transport_common.h"
#include "iec_ser_transport.h"
#include "iec_tcp_transport.h"
#include "esp_iec_slave.h"
#include "config.h"
#include "gpio_drv.h"

static const char *TAG = "iec_object.slave";

typedef struct
{
    iec_base_t base;
    iec_comm_mode_t cur_mode;
    iec_state_enum_t cur_state;
    uint8_t *frame;
    uint16_t length;
} iecs_object_t;

static iec_err_enum_t iecs_delete(iec_base_t *inst)
{
    iecs_object_t *iecs_obj = IEC_GET_OBJ_CTX(inst, iecs_object_t, base);
    iec_err_enum_t status = IEC_ENOERR;
    if (iecs_obj->cur_state == IEC_STATE_DISABLED) {
        if (IEC_OBJ(iecs_obj->base.transp_obj)->frm_delete) {
            // call destructor of the transport object
            IEC_OBJ(iecs_obj->base.transp_obj)->frm_delete(inst->transp_obj);
        }
        // delete the iec instance
        free(iecs_obj->base.descr.parent_name);
        CRITICAL_SECTION_CLOSE(inst->lock);
        free(inst);
        status = IEC_ENOERR;
    } else {
        ESP_LOGD(TAG, " need to disable %p object first.", (void *)iecs_obj);
        status = IEC_EILLSTATE;
    }
    iec_port_get_inst_counter_dec();
    return status;
}

static iec_err_enum_t iecs_enable(iec_base_t *inst)
{
    iecs_object_t *iecs_obj = IEC_GET_OBJ_CTX(inst, iecs_object_t, base);
    iec_err_enum_t status = IEC_ENOERR;
    CRITICAL_SECTION(inst->lock) {
        if (iecs_obj->cur_state == IEC_STATE_DISABLED) {
            /* Activate the protocol stack. */
            IEC_OBJ(iecs_obj->base.transp_obj)->frm_start(iecs_obj->base.transp_obj);
            iecs_obj->cur_state = IEC_STATE_ENABLED;
            status = IEC_ENOERR;
        } else {
            status = IEC_EILLSTATE;
        }
    }
    return status;
}

static iec_err_enum_t iecs_disable(iec_base_t *inst)
{
    iec_err_enum_t status = IEC_ENOERR;
    iecs_object_t *iecs_obj = IEC_GET_OBJ_CTX(inst, iecs_object_t, base);;
    CRITICAL_SECTION(inst->lock) {
        if (iecs_obj->cur_state == IEC_STATE_ENABLED) {
            IEC_OBJ(iecs_obj->base.transp_obj)->frm_stop(iecs_obj->base.transp_obj);
            iecs_obj->cur_state = IEC_STATE_DISABLED;
            status = IEC_ENOERR;
        } else if (iecs_obj->cur_state == IEC_STATE_DISABLED) {
            status = IEC_ENOERR;
        } else {
            status = IEC_EILLSTATE;
        }
    }
    return status;
}

static iec_err_enum_t iecs_poll(iec_base_t *inst)
{
    iecs_object_t *iecs_obj = IEC_GET_OBJ_CTX(inst, iecs_object_t, base);;

    iec_err_enum_t status = IEC_ENOERR;
    iec_event_t event;

    /* Check if the protocol stack is ready. */
    if (iecs_obj->cur_state != IEC_STATE_ENABLED) {
        return IEC_EILLSTATE;
    }

    /* Check if there is a event available. If not, return control to caller. Otherwise we will handle the event. */
    if (iec_port_event_get(IEC_OBJ(iecs_obj->base.port_obj), &event)) {
        switch(event.event) {
            case _EV_READY:
                ESP_LOGD(TAG, IEC_OBJ_FMT":EV_READY", IEC_OBJ_PARENT(inst));
                iec_port_event_res_release(IEC_OBJ(inst->port_obj));
                break;
            case _EV_FRAME_RECEIVED:
                ESP_LOGD(TAG, IEC_OBJ_FMT":EV_FRAME_RECEIVED", IEC_OBJ_PARENT(inst));
                iecs_obj->length = event.length;
                status = IEC_OBJ(inst->transp_obj)->frm_rcv(inst->transp_obj, &iecs_obj->frame, &iecs_obj->length);
                // Check if the frame is for us. If not ,send an error process event.
                if (status == IEC_ENOERR) {
                    (void)iec_port_event_post(IEC_OBJ(inst->port_obj), _EVENT(_EV_EXECUTE | _EV_TRANS_START));
                    ESP_LOG_BUFFER_HEX_LEVEL(IEC_STR_CAT(inst->descr.parent_name, ":IEC_RECV"), iecs_obj->frame,
                                             (uint16_t)iecs_obj->length, ESP_LOG_DEBUG);
                }
                break;
            case _EV_EXECUTE:
                if (IEC_OBJ(inst->transp_obj)->frm_send != NULL)
                    status = IEC_OBJ(inst->transp_obj)->frm_send(inst->transp_obj, iecs_obj->frame, iecs_obj->length);
                ESP_LOGD(TAG, IEC_OBJ_FMT":EV_EXECUTE", IEC_OBJ_PARENT(inst));
                break;
            case _EV_FRAME_TRANSMIT:
                ESP_LOGD(TAG, IEC_OBJ_FMT":EV_FRAME_TRANSMIT", IEC_OBJ_PARENT(inst));
                break;
            case _EV_FRAME_SENT:
                ESP_LOGD(TAG, IEC_OBJ_FMT":EV_MASTER_FRAME_SENT", IEC_OBJ_PARENT(inst));
                break;
            default:
                ESP_LOGD(TAG, IEC_OBJ_FMT": Unexpected event triggered 0x%02x.", IEC_OBJ_PARENT(inst), (int)event.event);
                break;
        }
    } else {
        // Something went wrong and task unblocked but there are no any correct events set
        ESP_LOGD(TAG, IEC_OBJ_FMT": Unexpected event triggered 0x%02x, timeout?", IEC_OBJ_PARENT(inst), (int)event.event);
        status = IEC_EILLSTATE;
    }
    return status;
}

iec_err_enum_t iecs_ser_create(iec_serial_opts_t *ser_opts, void **in_out_obj)
{
    iec_err_enum_t ret = IEC_ENOERR;
    IEC_RETURN_ON_FALSE(ser_opts, IEC_EINVAL, TAG, "invalid options for the instance.");
    IEC_RETURN_ON_FALSE((ser_opts->mode == IEC_SER), IEC_EILLSTATE, TAG, "incorrect mode != RTU.");
    iecs_object_t *iecs_obj = NULL;
    iecs_obj = (iecs_object_t *)calloc(1, sizeof(iecs_object_t));
    IEC_GOTO_ON_FALSE((iecs_obj), IEC_EILLSTATE, error, TAG, "no mem for iec slave instance.");
    CRITICAL_SECTION_INIT(iecs_obj->base.lock);
    iecs_obj->cur_state = IEC_STATE_NOT_INITIALIZED;
    iecs_obj->base.delete = iecs_delete;
    iecs_obj->base.enable = iecs_enable;
    iecs_obj->base.disable = iecs_disable;
    iecs_obj->base.poll = iecs_poll;
    iecs_obj->base.descr.parent = *in_out_obj;
    iecs_obj->base.descr.is_master = false;
    iecs_obj->base.descr.obj_name = (char *)TAG;
    iecs_obj->base.descr.inst_index = iec_port_get_inst_counter_inc();
    int res = asprintf(&iecs_obj->base.descr.parent_name, "iecs_rtu@%p", *in_out_obj);
    IEC_GOTO_ON_FALSE((res), IEC_EILLSTATE, error,
                      TAG, "name alloc fail, err: %d", (int)res);
    iec_trans_base_t *transp_obj = (iec_trans_base_t *)iecs_obj;
    ret = iecs_ser_transp_create(ser_opts, (void **)&transp_obj);
    IEC_GOTO_ON_FALSE((transp_obj && (ret == IEC_ENOERR)), IEC_EILLSTATE, error,
                      TAG, "transport creation, err: %d", (int)ret);
    iecs_obj->cur_mode = ser_opts->mode;
    iecs_obj->cur_state = IEC_STATE_DISABLED;
    transp_obj->get_tx_frm(transp_obj, (uint8_t **)&iecs_obj->frame);
    iecs_obj->base.port_obj = transp_obj->port_obj;
    iecs_obj->base.transp_obj = transp_obj;
    *in_out_obj = (void *)&(iecs_obj->base);
    ESP_LOGD(TAG, "created object %s", iecs_obj->base.descr.parent_name);
    return IEC_ENOERR;

error:
    if (transp_obj)
    {
        iecs_ser_transp_delete(transp_obj);
    }
    free(iecs_obj->base.descr.parent_name);
    CRITICAL_SECTION_CLOSE(iecs_obj->base.lock);
    free(iecs_obj);
    iec_port_get_inst_counter_dec();
    return ret;
}

iec_err_enum_t iecs_tcp_create(iec_tcp_opts_t *tcp_opts, void **in_out_obj)
{
    iec_err_enum_t ret = IEC_ENOERR;
    IEC_RETURN_ON_FALSE(tcp_opts, IEC_EINVAL, TAG, "invalid options for the instance.");
    IEC_RETURN_ON_FALSE((tcp_opts->mode == IEC_TCP), IEC_EILLSTATE, TAG, "incorrect mode != TCP.");
    iecs_object_t *iecs_obj = NULL;
    iecs_obj = (iecs_object_t *)calloc(1, sizeof(iecs_object_t));
    IEC_GOTO_ON_FALSE((iecs_obj), IEC_EILLSTATE, error, TAG, "no mem for iec slave instance.");
    CRITICAL_SECTION_INIT(iecs_obj->base.lock);
    iecs_obj->cur_state = IEC_STATE_NOT_INITIALIZED;
    iecs_obj->base.delete = iecs_delete;
    iecs_obj->base.enable = iecs_enable;
    iecs_obj->base.disable = iecs_disable;
    iecs_obj->base.poll = iecs_poll;
    iecs_obj->base.descr.parent = *in_out_obj;
    iecs_obj->base.descr.is_master = false;
    iecs_obj->base.descr.obj_name = (char *)TAG;
    iecs_obj->base.descr.inst_index = iec_port_get_inst_counter_inc();
    int res = asprintf(&iecs_obj->base.descr.parent_name, "iecs_tcp@%p", *in_out_obj);
    IEC_GOTO_ON_FALSE((res), IEC_EILLSTATE, error,
                      TAG, "name alloc fail, err: %d", (int)res);
    iec_trans_base_t *transp_obj = (iec_trans_base_t *)iecs_obj;
    ret = iecs_tcp_transp_create(tcp_opts, (void **)&transp_obj);
    IEC_GOTO_ON_FALSE((transp_obj && (ret == IEC_ENOERR)), IEC_EILLSTATE, error,
                      TAG, "transport creation, err: %d", (int)ret);
    iecs_obj->cur_mode = tcp_opts->mode;
    iecs_obj->cur_state = IEC_STATE_DISABLED;
    transp_obj->get_tx_frm(transp_obj, (uint8_t **)&iecs_obj->frame);
    iecs_obj->base.port_obj = transp_obj->port_obj;
    iecs_obj->base.transp_obj = transp_obj;
    *in_out_obj = (void *)&(iecs_obj->base);
    ESP_LOGD(TAG, "created object %s", iecs_obj->base.descr.parent_name);
    return IEC_ENOERR;

error:
    if (transp_obj)
    {
        iecs_tcp_transp_delete(transp_obj);
    }
    free(iecs_obj->base.descr.parent_name);
    CRITICAL_SECTION_CLOSE(iecs_obj->base.lock);
    free(iecs_obj);
    iec_port_get_inst_counter_dec();
    return ret;
}

bool iec_slave_is_connected(void)
{
    bool state;
    if (slave_select == SLAVE_IEC_104_TCP)
        state = iec_socket_is_connected();
    else
        state = is_iec_ser_start();
    set_slave_state_led(state);
    return state;
}