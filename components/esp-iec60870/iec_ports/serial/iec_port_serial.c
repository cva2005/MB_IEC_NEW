/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdatomic.h>
#include "esp_timer.h"
#include "iec_common.h"
#include "iec_port_common.h"
#include "iec_config.h"
#include "iec_port_serial_common.h"

/* ----------------------- Defines ------------------------------------------*/
#define IEC_SERIAL_RX_SEMA_TOUT_MS   (1000)
#define IEC_SERIAL_RX_SEMA_TOUT      (pdMS_TO_TICKS(IEC_SERIAL_RX_SEMA_TOUT_MS))
#define IEC_SERIAL_RX_FLUSH_RETRY    (2)
#define IEC_QUEUE_LENGTH             (20)
#define IEC_SERIAL_TOUT              (3)
#define IEC_SERIAL_TX_TOUT_TICKS     (pdMS_TO_TICKS(400))
#define IEC_SERIAL_TASK_STACK_SIZE   (CONFIG_FIEC_PORT_TASK_STACK_SIZE)     
#define IEC_SERIAL_RX_TOUT_TICKS     (pdMS_TO_TICKS(100))
#define IEC_SER_PDU_SIZE_MIN         (2)
#define IEC_SERIAL_MIN_PATTERN_INTERVAL  (9)
#define IEC_SERIAL_MIN_POST_IDLE         (0)
#define IEC_SERIAL_MIN_PRE_IDLE          (0)

typedef struct
{
    iec_port_base_t base;
    // serial communication properties
    iec_serial_opts_t ser_opts;
    bool rx_state_en;
    bool tx_state_en;
    uint16_t recv_length;
    uint64_t send_time_stamp;
    uint64_t recv_time_stamp;
    uint32_t flags;
    bool enabled;
    QueueHandle_t uart_queue;           // A queue to handle UART event.
    TaskHandle_t  task_handle;          // UART task to handle UART event.
    SemaphoreHandle_t bus_sema_handle;   // Rx blocking semaphore handle
} iec_ser_port_t;

/* ----------------------- Static variables & functions ----------------------*/
static const char *TAG = "iec_port.serial";

static bool iec_port_ser_bus_sema_init(iec_port_base_t *inst)
{
    iec_ser_port_t *port_obj = __containerof(inst, iec_ser_port_t, base);
    port_obj->bus_sema_handle = xSemaphoreCreateBinary();
    IEC_RETURN_ON_FALSE((port_obj->bus_sema_handle), false , TAG, 
                        "%s: RX semaphore create failure.", inst->descr.parent_name);
    return true;
}

static void iec_port_ser_bus_sema_close(iec_port_base_t *inst)
{
    iec_ser_port_t *port_obj = __containerof(inst, iec_ser_port_t, base);
    if (port_obj->bus_sema_handle) {
        vSemaphoreDelete(port_obj->bus_sema_handle);
        port_obj->bus_sema_handle = NULL;
    }
}

static bool iec_port_ser_bus_sema_take(iec_port_base_t *inst, uint32_t tm_ticks)
{
    BaseType_t status = pdTRUE;
    iec_ser_port_t *port_obj = __containerof(inst, iec_ser_port_t, base);
    status = xSemaphoreTake(port_obj->bus_sema_handle, tm_ticks );
    IEC_RETURN_ON_FALSE((status == pdTRUE), false , TAG, 
                        "%s,  rx semaphore take failure.", inst->descr.parent_name);
    ESP_LOGV(TAG, "%s: take RX semaphore (%" PRIu32" ticks).", inst->descr.parent_name, tm_ticks);
    return true;
}

static void iec_port_ser_bus_sema_release(iec_port_base_t *inst)
{
    BaseType_t status = pdFALSE;
    iec_ser_port_t *port_obj = __containerof(inst, iec_ser_port_t, base);
    status = xSemaphoreGive(port_obj->bus_sema_handle);
    if (status != pdTRUE) {
        ESP_LOGD(TAG, "%s,  rx semaphore is free.", inst->descr.parent_name);
    }
}

static bool iec_port_ser_bus_sema_is_busy(iec_port_base_t *inst)
{
    BaseType_t status = pdFALSE;
    iec_ser_port_t *port_obj = __containerof(inst, iec_ser_port_t, base);
    status = (uxSemaphoreGetCount(port_obj->bus_sema_handle) == 0) ? true : false;
    return status;
}

static void iec_port_ser_rx_flush(iec_port_base_t *inst)
{
    size_t size = 1;
    esp_err_t err = ESP_OK;
    iec_ser_port_t *port_obj = __containerof(inst, iec_ser_port_t, base);
    for (int cnt = 0; (cnt < IEC_SERIAL_RX_FLUSH_RETRY) && size; cnt++) {
        err = uart_get_buffered_data_len(port_obj->ser_opts.port, &size);
        IEC_RETURN_ON_FALSE((err == ESP_OK), ; , TAG, 
                                "%s, iec flush serial fail, error = 0x%x.", inst->descr.parent_name, (int)err);
        BaseType_t status = xQueueReset(port_obj->uart_queue);
        if (status) {
            err = uart_flush_input(port_obj->ser_opts.port);
            IEC_RETURN_ON_FALSE((err == ESP_OK), ; , TAG, 
                                "%s, iec flush serial fail, error = 0x%x.", inst->descr.parent_name, (int)err);
        }
    }
}

void iec_port_ser_enable(iec_port_base_t *inst)
{
    iec_ser_port_t *port_obj = __containerof(inst, iec_ser_port_t, base);
    CRITICAL_SECTION (port_obj->base.lock) {
        atomic_store(&(port_obj->enabled), true);
        iec_port_ser_bus_sema_release(inst);
        ESP_LOGD(TAG, "%s, resume port.", port_obj->base.descr.parent_name);
        // Resume receiver task from known position
        vTaskResume(port_obj->task_handle);
    }
}

void iec_port_ser_disable(iec_port_base_t *inst)
{
    iec_ser_port_t *port_obj = __containerof(inst, iec_ser_port_t, base);
    CRITICAL_SECTION (port_obj->base.lock) {
        // Suspend port task by itself
        atomic_store(&(port_obj->enabled), false);
        ESP_LOGD(TAG, "%s, suspend port.", port_obj->base.descr.parent_name);
    }
}

// UART receive event task
static void iec_port_ser_task(void *p_args)
{
    iec_ser_port_t *port_obj = __containerof(p_args, iec_ser_port_t, base);
    uart_event_t event;
    IEC_RETURN_ON_FALSE(port_obj, ;, TAG, "%s, get serial instance fail.", port_obj->base.descr.parent_name);
    (void)iec_port_ser_rx_flush(&port_obj->base);
    while(1) {
        // Workaround to suspend task from known place to avoid dead lock when resume
        if (!atomic_load(&(port_obj->enabled))) {
            ESP_LOGI(TAG, "%s, suspend port from task.", port_obj->base.descr.parent_name);
            vTaskSuspend(NULL);
        }
        if (xQueueReceive(port_obj->uart_queue, (void *)&event, IEC_SERIAL_RX_TOUT_TICKS)) {
            ESP_LOGD(TAG, "%s, UART[%d] event:", port_obj->base.descr.parent_name, port_obj->ser_opts.port);
            switch(event.type) {
                case UART_DATA:
                    ESP_LOGD(TAG, "%s, data event, len: %d.", port_obj->base.descr.parent_name, (int)event.size);
                    // This flag set in the event means that no more
                    // data received during configured timeout and UART TOUT feature is triggered
                    if (event.timeout_flag) {
                        // If bus is busy or fragmented data is received, then flush buffer
                        if (iec_port_ser_bus_sema_is_busy(&port_obj->base)) {
                            iec_port_ser_rx_flush(&port_obj->base);
                            break;
                        }
                        uart_get_buffered_data_len(port_obj->ser_opts.port, (unsigned int*)&event.size);
                        port_obj->recv_length = (event.size < IEC_BUFFER_SIZE) ? event.size : IEC_BUFFER_SIZE;
                        if (event.size <= IEC_SER_PDU_SIZE_MIN) {
                            ESP_LOGD(TAG, "%s, drop short packet %d byte(s)", port_obj->base.descr.parent_name, (int)event.size);
                            (void)iec_port_ser_rx_flush(&port_obj->base);
                            break;
                        }
                        // New frame is received, send an event to main FSM to read it into receiver buffer
                        iec_port_event_post(&port_obj->base, _EVENT(_EV_FRAME_RECEIVED, port_obj->recv_length, NULL, 0));
                        ESP_LOGD(TAG, "%s, frame %d bytes is ready.", port_obj->base.descr.parent_name, (int)port_obj->recv_length);
                    }
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGD(TAG, "%s, hw fifo overflow.", port_obj->base.descr.parent_name);
                    xQueueReset(port_obj->uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGD(TAG, "%s, ring buffer full.", port_obj->base.descr.parent_name);
                    (void)iec_port_ser_rx_flush(&port_obj->base);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGD(TAG, "%s, uart rx break.", port_obj->base.descr.parent_name);
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGD(TAG, "%s, uart parity error.", port_obj->base.descr.parent_name);
                    (void)iec_port_ser_rx_flush(&port_obj->base);
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGD(TAG, "%s, uart frame error.", port_obj->base.descr.parent_name);
                    (void)iec_port_ser_rx_flush(&port_obj->base);
                    break;
                default:
                    ESP_LOGD(TAG, "%s, uart event type: %d.", port_obj->base.descr.parent_name, (int)event.type);
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}

iec_err_enum_t iec_port_ser_create(iec_serial_opts_t *ser_opts, iec_port_base_t **in_out_obj)
{
    iec_ser_port_t *pserial = NULL;
    esp_err_t err = ESP_OK;
    __attribute__((unused)) iec_err_enum_t ret = IEC_EILLSTATE;
    pserial = (iec_ser_port_t*)calloc(1, sizeof(iec_ser_port_t));
    IEC_RETURN_ON_FALSE((pserial && in_out_obj), IEC_EILLSTATE, TAG, "iec serial port creation error.");
    CRITICAL_SECTION_INIT(pserial->base.lock);
    pserial->base.descr = ((iec_port_base_t*)*in_out_obj)->descr;
    ser_opts->data_bits = ((ser_opts->data_bits > UART_DATA_5_BITS) 
                                && (ser_opts->data_bits < UART_DATA_BITS_MAX)) 
                                ? ser_opts->data_bits : UART_DATA_8_BITS;
    // Keep the UART communication options
    pserial->ser_opts = *ser_opts;
    // Configure serial communication parameters
    uart_config_t uart_cfg = {
        .baud_rate = ser_opts->baudrate,
        .data_bits = ser_opts->data_bits,
        .parity = ser_opts->parity,
        .stop_bits = ser_opts->stop_bits,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 2,
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
        .source_clk = UART_SCLK_DEFAULT,
#else
        .source_clk = UART_SCLK_APB,
#endif
    };
    // Set UART config
    err = uart_param_config(pserial->ser_opts.port, &uart_cfg);
    IEC_GOTO_ON_FALSE((err == ESP_OK), IEC_EILLSTATE, error, TAG, 
                            "%s, iec config failure, uart_param_config() returned (0x%x).", pserial->base.descr.parent_name, (int)err);
    // Install UART driver, and get the queue.
    err = uart_driver_install(pserial->ser_opts.port, IEC_BUFFER_SIZE, IEC_BUFFER_SIZE,
                                    IEC_QUEUE_LENGTH, &pserial->uart_queue, IEC_PORT_SERIAL_ISR_FLAG);
    IEC_GOTO_ON_FALSE((err == ESP_OK), IEC_EILLSTATE, error, TAG,
                        "%s, iec serial driver failure, retuned (0x%x).", pserial->base.descr.parent_name, (int)err);
    err = uart_set_rx_timeout(pserial->ser_opts.port, IEC_SERIAL_TOUT);
    IEC_GOTO_ON_FALSE((err == ESP_OK), IEC_EILLSTATE, error, TAG,
                        "%s, iec serial set rx timeout failure, returned (0x%x).", pserial->base.descr.parent_name, (int)err);
    // Set always timeout flag to trigger timeout interrupt even after rx fifo full
    uart_set_always_rx_timeout(pserial->ser_opts.port, true);
    IEC_GOTO_ON_FALSE((iec_port_ser_bus_sema_init(&pserial->base)), IEC_EILLSTATE, error, TAG,
                                "%s, iec serial bus semaphore create fail.", pserial->base.descr.parent_name);
    // Suspend task on start and then resume when initialization is completed
    atomic_store(&(pserial->enabled), false);
    // Create a task to handle UART events
    BaseType_t status = xTaskCreatePinnedToCore(iec_port_ser_task, "port_serial_task",
                                                    IEC_SERIAL_TASK_STACK_SIZE,
                                                    &pserial->base, CONFIG_FIEC_PORT_TASK_PRIO,
                                                    &pserial->task_handle, CONFIG_FIEC_PORT_TASK_AFFINITY);
    // Force exit from function with failure
    IEC_GOTO_ON_FALSE((status == pdPASS), IEC_EILLSTATE, error, TAG,
                                "%s, iec stack serial task creation error, returned (0x%x).",
                                pserial->base.descr.parent_name, (int)status);
    *in_out_obj = &(pserial->base);
    ESP_LOGD(TAG, "created object @%p", pserial);
    return IEC_ENOERR;

error:
    if (pserial) {
        if (pserial->task_handle) {
            vTaskDelete(pserial->task_handle);
        }
        uart_driver_delete(pserial->ser_opts.port);
        CRITICAL_SECTION_CLOSE(pserial->base.lock);
        iec_port_ser_bus_sema_close(&pserial->base);
    }
    free(pserial);
    return IEC_EILLSTATE;
}

void iec_port_ser_delete(iec_port_base_t *inst)
{
    iec_ser_port_t *port_obj = __containerof(inst, iec_ser_port_t, base);
    vTaskDelete(port_obj->task_handle);
    ESP_ERROR_CHECK(uart_driver_delete(port_obj->ser_opts.port));
    iec_port_ser_bus_sema_close(inst);
    CRITICAL_SECTION_CLOSE(inst->lock);
    free(port_obj);
}

bool iec_port_ser_recv_data(iec_port_base_t *inst, uint8_t **pp_ser_frame, uint16_t *p_ser_length)
{
    IEC_RETURN_ON_FALSE((pp_ser_frame && p_ser_length), false, TAG, "iec serial get buffer failure.");
    iec_ser_port_t *port_obj = __containerof(inst, iec_ser_port_t, base);
    uint16_t counter = *p_ser_length ? *p_ser_length : port_obj->recv_length;
    bool status = false;

    status = iec_port_ser_bus_sema_take(inst, pdMS_TO_TICKS(iec_port_timer_get_response_time_ms(inst)));
    if (status && counter && *pp_ser_frame && atomic_load(&(port_obj->enabled))) {
        // Read frame data from the ringbuffer of receiver
        counter = uart_read_bytes(port_obj->ser_opts.port, (uint8_t *)*pp_ser_frame,
                                    counter, IEC_SERIAL_RX_TOUT_TICKS);
        // Stop timer because the new data is received
        iec_port_timer_disable(inst);
        // Store the timestamp of received frame
        port_obj->recv_time_stamp = esp_timer_get_time();
        ESP_LOGD(TAG, "%s, received data: %d bytes.", inst->descr.parent_name, (int)counter);
        ESP_LOG_BUFFER_HEX_LEVEL(IEC_STR_CAT(inst->descr.parent_name, ":PORT_RECV"), (void *)*pp_ser_frame, counter, ESP_LOG_DEBUG);
        int64_t time_delta = (port_obj->recv_time_stamp > port_obj->send_time_stamp) ? 
                                (port_obj->recv_time_stamp - port_obj->send_time_stamp) :
                                (port_obj->send_time_stamp - port_obj->recv_time_stamp);
        ESP_LOGD(TAG, "%s, serial processing time[us] = %" PRId64, inst->descr.parent_name, time_delta);
        status = true;
        *p_ser_length = counter;
    } else {
        ESP_LOGE(TAG, "%s: junk data (%d bytes) received. ", inst->descr.parent_name, (int)counter);
    }
    *p_ser_length = counter;
    iec_port_ser_bus_sema_release(inst);
    return status;
}

bool iec_port_ser_send_data(iec_port_base_t *inst, uint8_t *p_ser_frame, uint16_t ser_length)
{
    bool res = false;
    int count = 0;
    iec_ser_port_t *port_obj = __containerof(inst, iec_ser_port_t, base);

    res = iec_port_ser_bus_sema_take(inst, pdMS_TO_TICKS(iec_port_timer_get_response_time_ms(inst)));
    if (res && p_ser_frame && ser_length && atomic_load(&(port_obj->enabled))) {
        // Flush buffer received from previous transaction
        iec_port_ser_rx_flush(inst);
        count = uart_write_bytes(port_obj->ser_opts.port, p_ser_frame, ser_length);
        // Waits while UART sending the packet
        esp_err_t status = uart_wait_tx_done(port_obj->ser_opts.port, IEC_SERIAL_TX_TOUT_TICKS);
        (void)iec_port_event_post(inst, _EVENT(_EV_FRAME_SENT));
        ESP_LOGD(TAG, "%s, tx buffer sent: (%d) bytes.", inst->descr.parent_name, (int)count);
        IEC_RETURN_ON_FALSE((status == ESP_OK), false, TAG, "%s, iec serial sent buffer failure.",
                                inst->descr.parent_name);
        ESP_LOG_BUFFER_HEX_LEVEL(IEC_STR_CAT(inst->descr.parent_name, ":PORT_SEND"),
                                    (void *)p_ser_frame, ser_length, ESP_LOG_DEBUG);
        port_obj->send_time_stamp = esp_timer_get_time();
    } else {
        ESP_LOGE(TAG, "%s, send fail state:%d, %p, %u. ", inst->descr.parent_name, (int)port_obj->tx_state_en, p_ser_frame, (unsigned)ser_length);
    }
    iec_port_ser_bus_sema_release(inst);
    return res;
}