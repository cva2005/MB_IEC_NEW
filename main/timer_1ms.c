#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "timer_1ms.h"
#include "sdkconfig.h"

RTC_DATA_ATTR static uint64_t time_ms = 1746199497621;
static const char* TAG = "example";

static void timer_1ms_callback(void *arg)
{
    atomic_fetch_add(&time_ms, 1);
}

void timer_1ms_init(void)
{
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = timer_1ms_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "timer_1ms"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet */

    /* Start the timer */
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000));
	ESP_LOGI(TAG, "Started timer, time since boot: %lld us", esp_timer_get_time());
}

uint64_t get_time_ms(void)
{
    return atomic_load(&time_ms);
}

void set_time_ms(uint64_t time_new)
{
    atomic_store(&time_ms, time_new);
}

void set_finish_time(uint32_t delay, stime_t *time)
{
    time->run = get_time_ms();
    time->del = delay;
}

bool is_time_out(stime_t *time)
{
    return ((get_time_ms() - time->run) > time->del);
}