/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "iec_ser_transport.h"
#include "iec_port_serial_common.h"
#include "esp_iec_slave.h"

static const char *TAG = "iec_transp.ser_slave";

typedef struct
{
    iec_trans_base_t base;
    iec_port_base_t *port_obj;

    uint8_t snd_buf[IEC_SER_PDU_SIZE_MAX]; // pdu_buf
    uint8_t rcv_buf[IEC_SER_PDU_SIZE_MAX];
    uint16_t snd_pdu_len;
    uint8_t *snd_buf_cur;
    uint16_t snd_buf_cnt;
    uint16_t rcv_buf_pos;
    volatile iec_timer_mode_enum_t cur_timer_mode;
    iec_ser_state_enum_t state;
} iecs_ser_transp_t;

iec_err_enum_t iecs_ser_transp_create(iec_serial_opts_t *ser_opts, void **in_out_inst);
static void iecs_ser_transp_start(iec_trans_base_t *inst);
static void iecs_ser_transp_stop(iec_trans_base_t *inst);
static iec_err_enum_t iecs_ser_transp_receive(iec_trans_base_t *inst, uint8_t **frame_ptr_buf, uint16_t *len_buf);
static iec_err_enum_t iecs_ser_transp_send(iec_trans_base_t *inst, const uint8_t *frame_ptr, uint16_t len);
static bool iecs_ser_transp_rcv_fsm(iec_trans_base_t *inst);
static bool iecs_ser_transp_snd_fsm(iec_trans_base_t *inst);
static bool iecs_ser_transp_timer_expired(void *inst);
static void iecs_ser_transp_get_snd_buf(iec_trans_base_t *inst, uint8_t **frame_ptr_buf);
void iecs_ser_transp_get_rcv_buf(iec_trans_base_t *inst, uint8_t **frame_ptr_buf);
static uint16_t iecs_ser_transp_get_snd_len(iec_trans_base_t *inst);
static void iecs_ser_transp_set_snd_len(iec_trans_base_t *inst, uint16_t snd_pdu_len);
bool iecs_ser_transp_delete(iec_trans_base_t *inst);

iec_err_enum_t iecs_ser_transp_create(iec_serial_opts_t *ser_opts, void **in_out_inst)
{
    IEC_RETURN_ON_FALSE((ser_opts && in_out_inst), IEC_EINVAL, TAG, "invalid options for the instance.");
    iec_err_enum_t ret = IEC_ENOERR;
    iecs_ser_transp_t *transp = NULL;
    transp = (iecs_ser_transp_t *)calloc(1, sizeof(iecs_ser_transp_t));
    IEC_RETURN_ON_FALSE(transp, IEC_EILLSTATE, TAG, "no mem for ser slave transport instance.");
    CRITICAL_SECTION_INIT(transp->base.lock);
    CRITICAL_SECTION_LOCK(transp->base.lock);
    transp->base.frm_rcv = iecs_ser_transp_receive;
    transp->base.frm_send = iecs_ser_transp_send;
    transp->base.frm_start = iecs_ser_transp_start;
    transp->base.frm_stop = iecs_ser_transp_stop;
    transp->base.get_rx_frm = iecs_ser_transp_get_rcv_buf;
    transp->base.get_tx_frm = iecs_ser_transp_get_snd_buf;
    transp->base.frm_delete = iecs_ser_transp_delete;
    transp->base.frm_is_bcast = NULL;
    transp->base.descr = ((iec_port_base_t *)*in_out_inst)->descr;
    transp->base.descr.obj_name = (char *)TAG;
    iec_port_base_t *port_obj = (iec_port_base_t *)*in_out_inst;
    ret = iec_port_ser_create(ser_opts, &port_obj);
    IEC_GOTO_ON_FALSE((ret == IEC_ENOERR), IEC_EPORTERR, error, TAG, "serial port creation, err: %d", ret);
    ret = iec_port_timer_create(port_obj, IEC_RTU_GET_T35_VAL(ser_opts->baudrate));
    IEC_GOTO_ON_FALSE((ret == IEC_ENOERR), IEC_EPORTERR, error, TAG, "timer port creation, err: %d", ret);
    ret = iec_port_event_create(port_obj);
    IEC_GOTO_ON_FALSE((ret == IEC_ENOERR), IEC_EPORTERR, error, TAG, "event port creation, err: %d", ret);
    transp->base.port_obj = port_obj;
    // Set callback function pointer for the timer
    port_obj->cb.tmr_expired = iecs_ser_transp_timer_expired;
    port_obj->cb.tx_empty = NULL;
    port_obj->cb.byte_rcvd = NULL;
    port_obj->arg = (void *)transp;
    transp->port_obj = port_obj;
    *in_out_inst = &(transp->base);
    ESP_LOGD(TAG, "created %s object @%p", TAG, transp);
    CRITICAL_SECTION_UNLOCK(transp->base.lock);
    return IEC_ENOERR;

error:
    if (port_obj) {
        free(port_obj->event_obj);
        free(port_obj->timer_obj);
    }
    free(port_obj);
    CRITICAL_SECTION_CLOSE(transp->base.lock);
    free(transp);
    return ret;
}


bool iecs_ser_transp_delete(iec_trans_base_t *inst)
{
    iecs_ser_transp_t *transp = __containerof(inst, iecs_ser_transp_t, base);
    CRITICAL_SECTION(inst->lock) {
        iec_port_ser_delete(transp->base.port_obj);
        iec_port_timer_delete(transp->base.port_obj);
        iec_port_event_delete(transp->base.port_obj);
    }
    CRITICAL_SECTION_CLOSE(inst->lock);
    free(transp);
    return true;
}

static void iecs_ser_transp_start(iec_trans_base_t *inst)
{
    iecs_ser_transp_t *transp = __containerof(inst, iecs_ser_transp_t, base);
    transp->state = IEC_SER_STATE_INIT;
    CRITICAL_SECTION(inst->lock) {
        iec_port_ser_enable(inst->port_obj);
        //iec_port_timer_enable(inst->port_obj);
    };
    (void)iec_port_event_post(transp->base.port_obj, _EVENT(_EV_READY));
}

static void iecs_ser_transp_stop(iec_trans_base_t *inst)
{
    CRITICAL_SECTION(inst->lock) {
        iec_port_ser_disable(inst->port_obj);
        iec_port_timer_disable(inst->port_obj);
    };
}

static iec_err_enum_t iecs_ser_transp_receive(iec_trans_base_t *inst, uint8_t **ppframe_buf, uint16_t *pbuf_len)
{
    if (!pbuf_len || !ppframe_buf || !pbuf_len) {
        return IEC_EIO;
    }
    
    iecs_ser_transp_t *transp = __containerof(inst, iecs_ser_transp_t, base);

    uint8_t *pbuf = (uint8_t *)transp->rcv_buf;
    uint16_t length = *pbuf_len;

    if (iec_port_ser_recv_data(inst->port_obj, &pbuf, &length) != false){
        if (ser_frame_parse(pbuf_len, &pbuf))
        {
            memcpy(*ppframe_buf, pbuf, *pbuf_len);
            return IEC_ENOERR;
        }
    }
    return IEC_EIO;
}

static iec_err_enum_t iecs_ser_transp_send(iec_trans_base_t *inst, const uint8_t *frame_ptr, uint16_t frame_len)
{
    if (iec_port_ser_send_data(inst->port_obj, (uint8_t *)frame_ptr, frame_len) == false)
        return IEC_EIO;
    return IEC_ENOERR;
}

__attribute__((unused))
static bool iecs_ser_transp_rcv_fsm(iec_trans_base_t *inst)
{
    return false;
}

__attribute__((unused))
static bool iecs_ser_transp_snd_fsm(iec_trans_base_t *inst)
{
    return false;
}

static bool iecs_ser_transp_timer_expired(void *inst)
{
    iecs_ser_transp_t *transp = __containerof(inst, iecs_ser_transp_t, base);
    bool need_poll = false;
    //iec_timer_mode_enum_t timer_mode = iec_port_get_cur_timer_mode(transp->base.port_obj);

    iec_port_timer_disable(transp->base.port_obj);
    
    return need_poll;
}

static void iecs_ser_transp_get_snd_buf(iec_trans_base_t *inst, uint8_t **frame_ptr_buf)
{
    iecs_ser_transp_t *transp = __containerof(inst, iecs_ser_transp_t, base);
    CRITICAL_SECTION(inst->lock) {
        *frame_ptr_buf = (uint8_t *)transp->snd_buf;
    }
}

void iecs_ser_transp_get_rcv_buf(iec_trans_base_t *inst, uint8_t **frame_ptr_buf)
{
    iecs_ser_transp_t *transp = __containerof(inst, iecs_ser_transp_t, base);
    CRITICAL_SECTION(inst->lock) {
        *frame_ptr_buf = (uint8_t *)&transp->rcv_buf[IEC_PDU_FUNC_OFF];
    }
}

__attribute__((unused))
static uint16_t iecs_ser_transp_get_snd_len(iec_trans_base_t *inst)
{
    iecs_ser_transp_t *transp = __containerof(inst, iecs_ser_transp_t, base);
    return transp->snd_buf_cnt;
}

__attribute__((unused))
static void iecs_ser_transp_set_snd_len(iec_trans_base_t *inst, uint16_t snd_pdu_len)
{
    iecs_ser_transp_t *transp = __containerof(inst, iecs_ser_transp_t, base);
    CRITICAL_SECTION(inst->lock) {
        transp->snd_buf_cnt = snd_pdu_len;
    }   
}