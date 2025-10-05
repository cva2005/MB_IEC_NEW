/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>

#include "iec_config.h"
#include "iec_types.h"
#include "iec_port_common.h"
#include "iec_port_types.h"

#include "esp_log.h"

#include "sdkconfig.h"

/* Common definitions */

#ifdef __cplusplus
extern "C" {
#endif

#if __has_include("esp_check.h")
#include "esp_check.h"

#define IEC_RETURN_ON_FALSE(a, err_code, tag, format, ...) ESP_RETURN_ON_FALSE(a, err_code, tag, format __VA_OPT__(,) __VA_ARGS__)
#define IEC_GOTO_ON_ERROR(x, goto_tag, log_tag, format, ...) ESP_GOTO_ON_ERROR(x, goto_tag, log_tag, format __VA_OPT__(,) __VA_ARGS__)
#define IEC_GOTO_ON_FALSE(a, err_code, goto_tag, log_tag, format, ...) ESP_GOTO_ON_FALSE(a, err_code, goto_tag, log_tag, format __VA_OPT__(,) __VA_ARGS__)

#else

// if cannot include esp_check then use custom check macro

#define IEC_RETURN_ON_FALSE(a, err_code, tag, format, ...) do {                                         \
        if (!(a)) {                                                                                    \
            ESP_LOGE(tag, "%s(%d): " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);        \
            return err_code;                                                                           \
        }                                                                                              \
} while(0)

#define IEC_GOTO_ON_ERROR(x, goto_tag, log_tag, format, ...) do {                                           \
        esp_err_t err_rc_ = (x);                                                                           \
        if (err_rc_ != ESP_OK) {                                                                           \
            ESP_LOGE(log_tag, "%s(%d): " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);        \
            ret = err_rc_;                                                                                 \
            goto goto_tag;                                                                                 \
        }                                                                                                  \
    } while(0)

#define IEC_GOTO_ON_FALSE(a, err_code, goto_tag, log_tag, format, ...) do {                                  \
        (void)log_tag;                                                                                      \
        if (!(a)) {                                                                                         \
            ESP_LOGE(log_tag, "%s(%d): " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);         \
            ret = (err_code);                                                                               \
            goto goto_tag;                                                                                  \
        }                                                                                                   \
    } while (0) 

#endif

#define IEC_CAT_BUF_SIZE (100)

#define IEC_STR_CAT(pref, message) (__extension__(                               \
{                                                                               \
    char buf##__FUNCTION__##__LINE__[IEC_CAT_BUF_SIZE];                          \
    strncpy(&(buf##__FUNCTION__##__LINE__)[0], pref, (IEC_CAT_BUF_SIZE - 1));    \
    strncat((buf##__FUNCTION__##__LINE__), message, (IEC_CAT_BUF_SIZE - 1));     \
    (&((buf##__FUNCTION__##__LINE__)[0]));                                      \
}                                                                               \
))

#define IEC_OBJ_FMT "%p"

#define IEC_GET_OBJ_CTX(pinst, type, base) (__extension__(   \
{                                                           \
    assert(pinst);                                          \
    ((type *)__containerof(pinst, type, base));             \
}                                                           \
))

#define IEC_OBJ(pinst) (__extension__( \
{                                           \
    assert(pinst);                          \
    ((typeof(pinst))(pinst));               \
}                                           \
))

#define IEC_OBJ_PARENT(pinst) (__extension__(   \
    {                                           \
        assert(pinst);                          \
        (((iec_obj_descr_t *)(pinst))->parent); \
    }))

#define IEC_BASE2PORT(pinst) (__extension__(     \
{                                               \
    assert(pinst);                              \
    assert(((iec_base_t *)pinst)->port_obj);     \
    (((iec_base_t *)pinst)->port_obj);           \
}                                               \
))

typedef struct _iec_base_t iec_base_t;
typedef struct _iec_trans_base_t iec_trans_base_t;
typedef struct _iec_port_base_t iec_port_base_t;
typedef struct _iec_obj_descr iec_obj_descr_t;

typedef iec_err_enum_t (*iec_delete_fp)(iec_base_t *inst);
typedef iec_err_enum_t (*iec_enable_fp)(iec_base_t *inst);
typedef iec_err_enum_t (*iec_disable_fp)(iec_base_t *inst);
typedef iec_err_enum_t (*iec_poll_fp)(iec_base_t *inst);

typedef enum
{
    IEC_STATE_ENABLED,
    IEC_STATE_DISABLED,
    IEC_STATE_NOT_INITIALIZED
} iec_state_enum_t;

struct _iec_base_t
{
    iec_obj_descr_t descr;
    _lock_t lock;                   // base object lock
    iec_trans_base_t *transp_obj;
    iec_port_base_t  *port_obj;
    iec_delete_fp delete;
    iec_enable_fp enable;
    iec_disable_fp disable;
    iec_poll_fp poll;
};

typedef struct iec_port_tcp_opts iec_tcp_opts_t;
typedef struct iec_port_serial_opts iec_serial_opts_t;

iec_err_enum_t iecs_ser_create(iec_serial_opts_t *ser_opts, void **in_out_obj);
iec_err_enum_t iecs_tcp_create(iec_tcp_opts_t *tcp_opts, void **in_out_obj);

#ifdef __cplusplus
}
#endif