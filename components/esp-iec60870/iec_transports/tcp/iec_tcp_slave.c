#include "iec_tcp_transport.h"
#include "iec_port_tcp_common.h"
#include "esp_iec_slave.h"

static const char *TAG = "iec_transp.tcp_slave";

typedef struct
{
    iec_trans_base_t base;
    iec_port_base_t *port_obj;
    uint8_t recv_buf[IEC_TCP_BUF_SIZE];
    uint8_t send_buf[IEC_TCP_BUF_SIZE];
    iec_tcp_state_enum_t state;
    uint16_t snd_pdu_len;
} iecs_tcp_transp_t;

static void iecs_tcp_transp_start(iec_trans_base_t *inst);
static void iecs_tcp_transp_stop(iec_trans_base_t *inst);
static iec_err_enum_t iecs_tcp_transp_receive(iec_trans_base_t *inst, uint8_t **frame_ptr_buf, uint16_t *pbuf_len);
static iec_err_enum_t iecs_tcp_transp_send(iec_trans_base_t *inst, const uint8_t *frame_ptr, uint16_t len);
static void iecs_tcp_transp_get_rcv_buf(iec_trans_base_t *inst, uint8_t **frame_ptr_buf);
static void iecs_tcp_transp_get_snd_buf(iec_trans_base_t *inst, uint8_t **frame_ptr_buf);
bool iecs_tcp_transp_delete(iec_trans_base_t *inst);
static bool iecs_tcp_transp_timer_expired(void *inst);

iec_err_enum_t iecs_tcp_transp_create(iec_tcp_opts_t *tcp_opts, void **in_out_inst)
{
    iec_err_enum_t ret = IEC_ENOERR;
    iecs_tcp_transp_t *transp = NULL;
    transp = (iecs_tcp_transp_t *)calloc(1, sizeof(iecs_tcp_transp_t));
    IEC_RETURN_ON_FALSE(transp, IEC_EILLSTATE, TAG, "no mem for instance.");
    CRITICAL_SECTION_INIT(transp->base.lock);
    CRITICAL_SECTION_LOCK(transp->base.lock);
    transp->base.frm_rcv = iecs_tcp_transp_receive;
    transp->base.frm_send = NULL/*iecs_tcp_transp_send*/;
    transp->base.frm_start = iecs_tcp_transp_start;
    transp->base.frm_stop = iecs_tcp_transp_stop;
    transp->base.get_rx_frm = iecs_tcp_transp_get_rcv_buf;
    transp->base.get_tx_frm = iecs_tcp_transp_get_snd_buf;
    transp->base.frm_delete = iecs_tcp_transp_delete;
    transp->base.frm_is_bcast = NULL;
    // Copy parent object descriptor
    transp->base.descr = ((iec_port_base_t *)*in_out_inst)->descr;
    transp->base.descr.obj_name = (char *)TAG;
    iec_port_base_t *port_obj = (iec_port_base_t *)*in_out_inst;
    ret = iecs_port_tcp_create(tcp_opts, &port_obj);
    IEC_GOTO_ON_FALSE((ret == IEC_ENOERR), IEC_EPORTERR, error, TAG, "tcp port creation, err: %d", ret);
    ret = iec_port_timer_create(port_obj, IEC_TCP_TIMEOUT_MS * IEC_TIMER_TICS_PER_MS);
    IEC_GOTO_ON_FALSE((ret == IEC_ENOERR), IEC_EPORTERR, error, TAG, "timer port creation, err: %d", ret);
    // Override default response time if defined
    if (tcp_opts->response_tout_ms) {
        iec_port_timer_set_response_time(port_obj, tcp_opts->response_tout_ms);
    }
    ret = iec_port_event_create(port_obj);
    IEC_GOTO_ON_FALSE((ret == IEC_ENOERR), IEC_EPORTERR, error, TAG, "event port creation, err: %d", ret);
    transp->base.port_obj = port_obj;
    // Set callback function pointer for the timer
    port_obj->cb.tmr_expired = iecs_tcp_transp_timer_expired;
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
    CRITICAL_SECTION_UNLOCK(transp->base.lock);
    CRITICAL_SECTION_CLOSE(transp->base.lock);
    free(transp);
    return ret;
}

bool iecs_tcp_transp_delete(iec_trans_base_t *inst)
{
    iecs_tcp_transp_t *transp = __containerof(inst, iecs_tcp_transp_t, base);
    // destroy method of port tcp slave is here
    CRITICAL_SECTION(inst->lock) {
        iecs_port_tcp_delete(inst->port_obj);
        iec_port_timer_delete(inst->port_obj);
        iec_port_event_delete(inst->port_obj);
    }
    CRITICAL_SECTION_CLOSE(inst->lock);
    free(transp);
    return true;
}

static void iecs_tcp_transp_start(iec_trans_base_t *inst)
{
    CRITICAL_SECTION(inst->lock) {
        iecs_port_tcp_enable(inst->port_obj);
        iec_port_timer_enable(inst->port_obj);
    };
    /* No special startup required for TCP. */
    (void)iec_port_event_post(inst->port_obj, _EVENT(_EV_READY));
}

static void iecs_tcp_transp_stop(iec_trans_base_t *inst)
{
    /* Make sure that no more clients are connected. */
    CRITICAL_SECTION(inst->lock) {
        iecs_port_tcp_disable(inst->port_obj);
        iec_port_timer_disable(inst->port_obj);
    };
}

static iec_err_enum_t iecs_tcp_transp_receive(iec_trans_base_t *inst, uint8_t **frame_ptr_buf, uint16_t *pbuf_len)
{
    if (!frame_ptr_buf || !*frame_ptr_buf || !pbuf_len)
        return IEC_EIO;
    uint16_t length;
    uint8_t *buff = get_tcp_buff_ptr();
    if (iecs_port_tcp_recv_data(inst->port_obj, &buff, &length) != false)
    {
        tcp_frame_ready(length);
        return IEC_ENOERR;
    }
    return IEC_EIO;
}

static iec_err_enum_t iecs_tcp_transp_send(iec_trans_base_t *inst, const uint8_t *pframe, uint16_t len)
{
    if (iecs_port_tcp_send_data(inst->port_obj, (uint8_t *)pframe, len) == false)
        return IEC_EIO;
    return IEC_ENOERR;
}

static bool iecs_tcp_transp_timer_expired(void *inst)
{
    iecs_tcp_transp_t *transp = __containerof(inst, iecs_tcp_transp_t, base);
    
    bool need_poll = false;
    iec_timer_mode_enum_t timer_mode = iec_port_get_cur_timer_mode(transp->base.port_obj);

    iec_port_timer_disable(transp->base.port_obj);

    switch(timer_mode) {
        case IEC_TMODE_T35:
            need_poll = iec_port_event_post(transp->base.port_obj, _EVENT(_EV_READY));
            ESP_EARLY_LOGD(TAG, "EV_READY");
            break;

        case IEC_TMODE_RESPOND_TIMEOUT:
            iec_port_event_set_err_type(transp->base.port_obj, _EV_ERROR_RESPOND_TIMEOUT);
            need_poll = iec_port_event_post(transp->base.port_obj, _EVENT(_EV_ERROR_PROCESS));
            ESP_EARLY_LOGD(TAG, "EV_ERROR_RESPOND_TIMEOUT");
            break;

        case IEC_TMODE_CONVERT_DELAY:
            /* If timer mode is convert delay, the master event then turns EV_MASTER_EXECUTE status. */
            need_poll = iec_port_event_post(transp->base.port_obj, _EVENT(_EV_EXECUTE));
            ESP_EARLY_LOGD(TAG, "IEC_TMODE_CONVERT_DELAY");
            break;

        default:
            need_poll = iec_port_event_post(transp->base.port_obj, _EVENT(_EV_READY));
            break;
    }
    
    return need_poll;
}

static void iecs_tcp_transp_get_rcv_buf(iec_trans_base_t *inst, uint8_t **frame_ptr_buf)
{
    iecs_tcp_transp_t *transp = __containerof(inst, iecs_tcp_transp_t, base);
    CRITICAL_SECTION(inst->lock) {
        *frame_ptr_buf = transp->recv_buf;
    }
}

static void iecs_tcp_transp_get_snd_buf(iec_trans_base_t *inst, uint8_t **frame_ptr_buf)
{
    iecs_tcp_transp_t *transp = __containerof(inst, iecs_tcp_transp_t, base);
    CRITICAL_SECTION(inst->lock) {
        *frame_ptr_buf = transp->send_buf;
    }
}