#include <stdbool.h>
#include <string.h>
#include <esp_log.h>
#include "config.h"
#include "arch.h"
#include "timer_1ms.h"

#define STORAGE_NAMESPACE "storage"
#define ARC_LEN         64
#define TO_MS_MUL       1000
#define ADD_TIME_10_SEC 10000

static const char *TAG = "arch.c";
static int next;
arc_rec_t ArcRec[ARC_LEN] = {0};
static uint32_t max_rtc_cnt = 0;

static esp_err_t save_nvs_rec(arc_rec_t new)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_open error: %d", err);
        goto err_exit;
    }
    char str[8];
    sprintf(str, "rtc%d", next);
    err = nvs_set_u32(nvs_handle, str, new.rtc_count);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_set_u32 error: %d", err);
        goto err_exit;
    }
    sprintf(str, "evt%d", next);
    err = nvs_set_u32(nvs_handle, str, new.arc_event);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_set_u32 error: %d", err);
        goto err_exit;
    }
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "nvs_commit error: %d", err);
err_exit:
    nvs_close(nvs_handle);
    return err;
}

esp_err_t init_events_arch(nvs_handle_t nvs_handle)
{
    esp_err_t err;
    for (int i = 0; i < ARC_LEN; i++)
    {
        char str[8];
        sprintf(str, "rtc%d", i);
        err = nvs_get_u32(nvs_handle, str, &ArcRec[i].rtc_count);
        if (err != ESP_OK)
            break;
        sprintf(str, "evt%d", i);
        err = nvs_get_u32(nvs_handle, str, &ArcRec[i].arc_event);
        if (err != ESP_OK)
            break;
    }
    if (err != ESP_OK)
        ESP_LOGE(TAG, "init_events_arch() error: %d", err);
    else
    {
        uint32_t arc_cnt;
        next = ARC_LEN - 1;
        for (int i = 0; i < ARC_LEN; i++)
        {
            arc_cnt = ArcRec[i].rtc_count;
            if (arc_cnt > max_rtc_cnt)
            {
                max_rtc_cnt = arc_cnt;
                next = i;
            }
        }
        uint64_t last_time = max_rtc_cnt;
        last_time *= TO_MS_MUL;
        if (get_time_ms() < last_time)
        {
            last_time += ADD_TIME_10_SEC;
            set_time_ms(last_time);
            if (nvs_set_u64(nvs_handle, "utc", last_time) == ESP_OK)
                nvs_commit(nvs_handle);
        }
        next++;
        if (next == ARC_LEN)
            next = 0;
    }
    return err;
}

esp_err_t clear_events_arch(void)
{
    memset(ArcRec, 0, sizeof(arc_rec_t) * ARC_LEN);
    next = 0;
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_open error: %d", err);
        goto err_exit;
    }
    for (int i = 0; i < ARC_LEN; i++)
    {
        char str[8];
        sprintf(str, "rtc%d", i);
        err = nvs_set_u32(nvs_handle, str, 0);
        if (err != ESP_OK)
            break;
        sprintf(str, "evt%d", i);
        err = nvs_set_u32(nvs_handle, str, 0);
        if (err != ESP_OK)
            break;
    }
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_set_u32 error: %d", err);
        goto err_exit;
    }
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_commit error: %d", err);
        goto err_exit;
    }
err_exit:
    nvs_close(nvs_handle);
    return err;
}

void save_arch_event(uint32_t sys_event)
{
    arc_rec_t rec;
    rec.arc_event = esp_reset_reason();
    rec.arc_event |= sys_event;
    rec.rtc_count = get_time_ms() / 1000;
    ArcRec[next] = rec;
    save_nvs_rec(rec);
    next++;
    if (next == ARC_LEN) next = 0;
}

char *get_arch_json(char *p)
{
    p += sprintf(p, "\"ArcT\":[");
    for (int i = 0; i < ARC_LEN; i++)
    {
        *p++ = '[';
        p += sprintf(p, "%lu,", ArcRec[i].rtc_count);
        p += sprintf(p, "%lu", ArcRec[i].arc_event);
        *p++ = ']';
        *p++ = ',';
    }
    p--;        // remove last comma
    *p++ = ']'; // end of array
    return p;
}

esp_err_t save_utc_copy(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_open error: %d", err);
        goto err_exit;
    }
    ESP_LOGI(TAG, "nvs utc = %llu", get_time_ms());
    err = nvs_set_u64(nvs_handle, "utc", get_time_ms());
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_set_u32 error: %d", err);
        goto err_exit;
    }
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "nvs_commit error: %d", err);
err_exit:
    nvs_close(nvs_handle);
    return err;
}

esp_err_t read_utc_copy(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_open error: %d", err);
        goto err_exit;
    }
    uint64_t utc;
    err = nvs_get_u64(nvs_handle, "utc", &utc);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_get_u64 error: %d", err);
        goto err_exit;
    }
    else
        set_time_ms(utc);
err_exit:
    nvs_close(nvs_handle);
    return err;
}