/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <string.h>
/*----------------------- Platform includes --------------------------------*/
#include "spinlock.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"

#include "iec_port_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define IEC_TIMER_TICS_PER_MS            (20UL)                         // Define number of timer reloads per 1 mS
#define IEC_TIMER_TICK_TIME_US           (1000 / IEC_TIMER_TICS_PER_MS) // 50uS = one discreet for timer
#define IEC_EVENT_QUEUE_TIMEOUT_MAX_MS   (3000)
#define IEC_EVENT_QUEUE_TIMEOUT          (pdMS_TO_TICKS(CONFIG_FIEC_EVENT_QUEUE_TIMEOUT))
#define IEC_EVENT_QUEUE_TIMEOUT_MAX      (pdMS_TO_TICKS(IEC_EVENT_QUEUE_TIMEOUT_MAX_MS))

int lock_obj(_lock_t *plock);
void unlock_obj(_lock_t *plock);

#define CRITICAL_SECTION_INIT(lock)   \
    do                                \
    {                                 \
        _lock_init((_lock_t *)&lock); \
    } while (0)

#define CRITICAL_SECTION_CLOSE(lock)   \
    do                                 \
    {                                  \
        _lock_close((_lock_t *)&lock); \
    } while (0)

#define CRITICAL_SECTION_LOCK(lock) \
    do                              \
    {                               \
        lock_obj((_lock_t *)&lock); \
    } while (0)

#define CRITICAL_SECTION_UNLOCK(lock) \
    do                                \
    {                                 \
        unlock_obj((_lock_t *)&lock); \
    } while (0)

#define CRITICAL_SECTION(lock) for (int st = lock_obj((_lock_t *)&lock); (st > 0); unlock_obj((_lock_t *)&lock), st = -1)

#define SPIN_LOCK_INIT(lock)        \
    do                              \
    {                               \
        spinlock_initialize(&lock); \
    } while (0)


#define CRITICAL_STORE(LOCK, PTR, VAL) \
__extension__ \
({  \
    __auto_type __atomic_ptr = (PTR); \
    __typeof__ ((void)0, *__atomic_ptr) __atomic_tmp = (VAL); \
    _lock_acquire((_lock_t *)&LOCK); \
    *__atomic_ptr = __atomic_tmp; \
    _lock_release((_lock_t *)&LOCK); \
    (__atomic_tmp); \
})

#define CRITICAL_LOAD(LOCK, PTR) \
__extension__ \
({  \
    __auto_type __atomic_ptr = (PTR); \
    __typeof__ ((void)0, *__atomic_ptr) __atomic_tmp; \
    _lock_acquire((_lock_t *)&LOCK); \
    __atomic_tmp = (*__atomic_ptr); \
    _lock_release((_lock_t *)&LOCK); \
    (__atomic_tmp); \
})

#define SPIN_LOCK_ENTER(lock)                           \
    do                                                  \
    {                                                   \
        spinlock_acquire(&lock, SPINLOCK_WAIT_FOREVER); \
    } while (0)

#define SPIN_LOCK_EXIT(lock)     \
    do                           \
    {                            \
        spinlock_release(&lock); \
    } while (0)

#define IEC_EVENT_REQ_MASK (EventBits_t)(_EV_MASTER_PROCESS_SUCCESS |       \
                                        _EV_MASTER_ERROR_RESPOND_TIMEOUT | \
                                        _EV_MASTER_ERROR_RECEIVE_DATA |    \
                                        _EV_MASTER_ERROR_EXECUTE_FUNCTION)

#define IEC_PORT_CHECK_EVENT(event, mask) (event & mask)
#define IEC_PORT_CLEAR_EVENT(event, mask) \
    do                                   \
    {                                    \
        event &= ~mask;                  \
    } while (0)

// concatenation of the two arguments
#define PP_CAT2(_1, _2) PP_CAT_(_1, _2)
#define PP_CAT_(_1, _2) _1##_2

#define PP_VA_NUM_ARGS(...) PP_VA_NUM_ARGS_(__VA_ARGS__, 4, 3, 2, 1)
#define PP_VA_NUM_ARGS_(_1, _2, _3, _4, N, ...) N

// Initialization of event structure using variadic parameters
#define _EVENT(...) PP_CAT2(_EVENT_, PP_VA_NUM_ARGS(__VA_ARGS__))(__VA_ARGS__)

#define _EVENT_1(_1) \
    (iec_event_t) { .event = _1 }
#define _EVENT_2(_1, _2) \
    (iec_event_t) { .event = _1, .length = _2 }
#define _EVENT_3(_1, _2, _3) \
    (iec_event_t) { .event = _1, .length = _2, .pdata = _3 }
#define _EVENT_4(_1, _2, _3, _4) \
    (iec_event_t) { .event = _1, .length = _2, .pdata = _3, .type = _4 }

typedef struct _iec_port_base_t iec_port_base_t;

typedef struct
{
    iec_port_base_t *iec_base;
} iec_common_iface_t;

//((iec_port_base_t *)(((iec_common_iface_t *)pctx)->iec_base)->lock);

#define IEC_OBJ_GET_LOCK(pctx) (__extension__(                  \
{                                                          \
    assert((pctx));                                        \
    iec_common_iface_t *iface = (iec_common_iface_t *)pctx;  \
    ((_lock_t)((iec_port_base_t *)(iface->iec_base))->lock); \
}))

typedef bool (*iec_port_cb_fp)(void *arg);

//!< port callback table for interrupts
typedef struct
{
    iec_port_cb_fp byte_rcvd;
    iec_port_cb_fp tx_empty;
    iec_port_cb_fp tmr_expired;
} iec_port_cb_t;

typedef struct iec_port_event_t iec_port_event_t;
typedef struct iec_port_timer_t iec_port_timer_t;
typedef struct _iec_obj_descr iec_obj_descr_t;

typedef struct _iec_frame_queue_entry
{
    uint16_t tid;  /*!< Transaction identifier (TID) for slave */
    uint16_t pid;  /*!< Protocol ID field of IECAP frame */
    uint16_t uid;  /*!< Slave unit ID (UID) field for IECAP frame  */
    uint8_t *pbuf; /*!< Points to the buffer for the frame */
    uint16_t len;  /*!< Length of the frame in the buffer */
    bool check;    /*!< Checked flag */
} iec_frame_entry_t;

struct _iec_port_base_t
{
    iec_obj_descr_t descr;
    _lock_t lock;
    iec_port_cb_t cb; //!< Port callbacks.
    void *arg;       //!< CB arg pointer.

    iec_port_event_t *event_obj;
    iec_port_timer_t *timer_obj;
};

// Port event functions
iec_err_enum_t iec_port_event_create(iec_port_base_t *port_obj);
bool iec_port_event_post(iec_port_base_t *inst, iec_event_t event);
bool iec_port_event_get(iec_port_base_t *inst, iec_event_t *event);
bool iec_port_event_res_take(iec_port_base_t *inst, uint32_t timeout);
void iec_port_event_res_release(iec_port_base_t *inst);
void iec_port_event_set_resp_flag(iec_port_base_t *inst, iec_event_enum_t event_mask);
void iec_port_event_set_err_type(iec_port_base_t *inst, iec_err_event_t event);
iec_err_event_t iec_port_event_get_err_type(iec_port_base_t *inst);
void iec_port_event_delete(iec_port_base_t *inst);
iec_err_enum_t iec_port_event_wait_req_finish(iec_port_base_t *inst);
uint64_t iec_port_get_trans_id(iec_port_base_t *inst);

// Port timer functions
iec_err_enum_t iec_port_timer_create(iec_port_base_t *inst, uint16_t t35_timer_ticks);
void iec_port_timer_disable(iec_port_base_t *inst);
void iec_port_timer_enable(iec_port_base_t *inst);
void iec_port_timer_respond_timeout_enable(iec_port_base_t *inst);
void iec_port_timer_convert_delay_enable(iec_port_base_t *inst);
void iec_port_set_cur_timer_mode(iec_port_base_t *inst, iec_timer_mode_enum_t tmr_mode);
iec_timer_mode_enum_t iec_port_get_cur_timer_mode(iec_port_base_t *inst);
void iec_port_timer_set_response_time(iec_port_base_t *inst, uint32_t resp_time_ms);
uint32_t iec_port_timer_get_response_time_ms(iec_port_base_t *inst);
void iec_port_timer_delay(iec_port_base_t *inst, uint16_t timeout_ms);
void iec_port_timer_delete(iec_port_base_t *inst);

// Common functions to track instance descriptors
void iec_port_set_inst_counter(uint32_t inst_counter);
uint32_t iec_port_get_inst_counter();
uint32_t iec_port_get_inst_counter_inc();
uint32_t iec_port_get_inst_counter_dec();

// Common queue functions
QueueHandle_t iec_queue_create(int queue_size);
void iec_queue_delete(QueueHandle_t queue);
void iec_queue_flush(QueueHandle_t queue);
bool iec_queue_is_empty(QueueHandle_t queue);
esp_err_t iec_queue_push(QueueHandle_t queue, void *pbuf, size_t len, iec_frame_entry_t *pframe);
ssize_t iec_queue_pop(QueueHandle_t queue, void *pbuf, size_t len, iec_frame_entry_t *pframe);

#ifdef __cplusplus
}
#endif