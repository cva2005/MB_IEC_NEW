/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*----------------------- Platform includes --------------------------------*/
#include <stdatomic.h>
#include "esp_idf_version.h"
#include "esp_attr.h"

#if __has_include("driver/gptimer.h")
#include "driver/gptimer.h"
#else
#include "driver/timer.h"
#endif

#include "esp_timer.h"
#include "esp_log.h"

#include "iec_port_common.h"
#include "iec_types.h"
#include "iec_config.h"
#include "iec_common.h"

/* ----------------------- Defines ----------------------------------------*/
struct iec_port_timer_t
{
    //spinlock_t spin_lock;
    esp_timer_handle_t timer_handle;
    uint16_t t35_ticks;
    _Atomic uint32_t response_time_ms;
    _Atomic bool timer_state;
    _Atomic uint16_t timer_mode;
};

/* ----------------------- Static variables ---------------------------------*/
static const char *TAG = "iec_port.timer";

/* ----------------------- Start implementation -----------------------------*/
iec_timer_mode_enum_t iec_port_get_cur_timer_mode(iec_port_base_t *inst);

static void IRAM_ATTR timer_alarm_cb(void *param)
{
    iec_port_base_t *inst = (iec_port_base_t *)param;
    if (inst->cb.tmr_expired && inst->arg) {
        inst->cb.tmr_expired(inst->arg); // Timer expired callback function
    }
    atomic_store(&(inst->timer_obj->timer_state), true);
    ESP_EARLY_LOGD(TAG, "timer mode: (%d) triggered", iec_port_get_cur_timer_mode(inst));
}

iec_err_enum_t iec_port_timer_create(iec_port_base_t *inst, uint16_t t35_timer_ticks)
{
    IEC_RETURN_ON_FALSE((t35_timer_ticks > 0), IEC_EILLSTATE, TAG,
                       "modbus timeout discreet is incorrect.");
    // IEC_RETURN_ON_FALSE((inst && !inst->timer_obj), IEC_EILLSTATE, TAG,
    //                    "modbus timer is already created.");
    iec_err_enum_t ret = IEC_EILLSTATE;
    inst->timer_obj = (iec_port_timer_t *)calloc(1, sizeof(iec_port_timer_t));
    IEC_GOTO_ON_FALSE((inst && inst->timer_obj), IEC_EILLSTATE, error, TAG, "iec timer allocation error.");
    inst->timer_obj->timer_handle = NULL;
    atomic_init(&(inst->timer_obj->timer_mode), IEC_TMODE_T35);
    atomic_init(&(inst->timer_obj->timer_state), false);
    // Set default response time according to kconfig
    atomic_init(&(inst->timer_obj->response_time_ms), CONFIG_IEC_MASTER_TIMEOUT_MS_RESPOND);
    // Save timer reload value for Modbus T35 period
    inst->timer_obj->t35_ticks = t35_timer_ticks;
    esp_timer_create_args_t timer_conf = {
        .callback = timer_alarm_cb,
        .arg = inst,
#if (IEC_TIMER_SUPPORTS_ISR_DISPATCH_METHOD && IEC_TIMER_USE_ISR_DISPATCH_METHOD)
        .dispatch_method = ESP_TIMER_ISR,
#else
        .dispatch_method = ESP_TIMER_TASK,
#endif
        .name = "IEC_T35timer"
    };
    // Create Modbus timer
    esp_err_t err = esp_timer_create(&timer_conf, &(inst->timer_obj->timer_handle));
    IEC_GOTO_ON_FALSE((err == ESP_OK), IEC_EILLSTATE, error, TAG, "iec timer creation error.");
    ESP_LOGD(TAG, "initialized %s object @%p", TAG, inst->timer_obj);
    return IEC_ENOERR;

error:
    if (inst && inst->timer_obj && inst->timer_obj->timer_handle)
    {
        esp_timer_delete(inst->timer_obj->timer_handle);
    }
    free(inst->timer_obj);
    inst->timer_obj = NULL;
    return ret;
}

void iec_port_timer_delete(iec_port_base_t *inst)
{
    // Delete active timer
    if (inst->timer_obj)
    {
        if (inst->timer_obj->timer_handle)
        {
            esp_timer_stop(inst->timer_obj->timer_handle);
            esp_timer_delete(inst->timer_obj->timer_handle);
        }
        free(inst->timer_obj);
        inst->timer_obj = NULL;
    }
}

void iec_port_timer_us(iec_port_base_t *inst, uint64_t timeout_us)
{
    IEC_RETURN_ON_FALSE((inst && inst->timer_obj->timer_handle), ;, TAG, "timer is not initialized.");
    IEC_RETURN_ON_FALSE((timeout_us > 0), ;, TAG,
                        "%s, incorrect tick value for timer = (%" PRId64 ").", inst->descr.parent_name, timeout_us);
    esp_timer_stop(inst->timer_obj->timer_handle);
    esp_timer_start_once(inst->timer_obj->timer_handle, timeout_us);
    atomic_store(&(inst->timer_obj->timer_state), false);
}


inline void iec_port_set_cur_timer_mode(iec_port_base_t *inst, iec_timer_mode_enum_t tmr_mode)
{
    atomic_store(&(inst->timer_obj->timer_mode), tmr_mode);
}

inline iec_timer_mode_enum_t iec_port_get_cur_timer_mode(iec_port_base_t *inst)
{
    return atomic_load(&(inst->timer_obj->timer_mode));
}

void iec_port_timer_enable(iec_port_base_t *inst)
{
    uint64_t tout_us = (inst->timer_obj->t35_ticks * IEC_TIMER_TICK_TIME_US);

    // Set current timer mode, don't change it.
    iec_port_set_cur_timer_mode(inst, IEC_TMODE_T35);
    // Set timer alarm
    iec_port_timer_us(inst, tout_us);
    ESP_LOGD(TAG, "%s, start timer (%" PRIu64 ").", inst->descr.parent_name, tout_us);
}

void iec_port_timer_convert_delay_enable(iec_port_base_t *inst)
{
    // Covert time in milliseconds into ticks
    uint64_t tout_us = (CONFIG_IEC_MASTER_DELAY_MS_CONVERT * 1000);

    // Set current timer mode
    iec_port_set_cur_timer_mode(inst, IEC_TMODE_CONVERT_DELAY);
    ESP_LOGD(TAG, "%s, convert delay enable.", inst->descr.parent_name);
    iec_port_timer_us(inst, tout_us);
}

void iec_port_timer_respond_timeout_enable(iec_port_base_t *inst)
{
    uint64_t tout_us = (inst->timer_obj->response_time_ms * 1000);

    iec_port_set_cur_timer_mode(inst, IEC_TMODE_RESPOND_TIMEOUT);
    ESP_LOGD(TAG, "%s, respond enable timeout (%u).", 
                inst->descr.parent_name, (unsigned)iec_port_timer_get_response_time_ms(inst));
    iec_port_timer_us(inst, tout_us);
}

void iec_port_timer_delay(iec_port_base_t *inst, uint16_t timeout_ms)
{
    uint64_t tout_us = (timeout_ms * 1000);
    iec_port_timer_us(inst, tout_us);
}

void iec_port_timer_disable(iec_port_base_t *inst)
{
    // Disable timer alarm
    esp_err_t err = esp_timer_stop(inst->timer_obj->timer_handle);
    if (err != ESP_OK)
    {
        if (!esp_timer_is_active(inst->timer_obj->timer_handle))
        {
            ESP_EARLY_LOGD(TAG, "%s, timer stop, returns %d.", inst->descr.parent_name, (int)err);
        }
    }
}

void iec_port_timer_set_response_time(iec_port_base_t *inst, uint32_t resp_time_ms)
{
    atomic_store(&(inst->timer_obj->response_time_ms), resp_time_ms);
}

uint32_t iec_port_timer_get_response_time_ms(iec_port_base_t *inst)
{
    return atomic_load(&(inst->timer_obj->response_time_ms));
}
