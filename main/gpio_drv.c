/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define GPIO_OUTPUT_IO_0    16
#define GPIO_OUTPUT_IO_1    2
#define GPIO_OUTPUT_PIN_SEL  ((1ULL << GPIO_OUTPUT_IO_0) | (1ULL << GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0     36
#define GPIO_INPUT_IO_1     39
#define FACTORY_BUTTON      34
#define GPIO_INPUT_PIN_SEL  ((1ULL << GPIO_INPUT_IO_0) |\
                             (1ULL << GPIO_INPUT_IO_1) |\
                             (1ULL << FACTORY_BUTTON))
#define DEBOUNCE 100

void gpio_init(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {0};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}

bool is_web_cfg(void)
{
    int db_count = DEBOUNCE;
    while (db_count--)
    {
        if (gpio_get_level(GPIO_INPUT_IO_1))
            return false;
    }
    return true;
}

bool is_factory_button(void)
{
    int db_count = DEBOUNCE;
    while (db_count--)
    {
        if (gpio_get_level(FACTORY_BUTTON))
            return false;
    }
    return true;
}
bool is_rollback_on(void)
{
    int db_count = DEBOUNCE;
    while (db_count--)
    {
        if (gpio_get_level(GPIO_INPUT_IO_0))
            return false;
    }
    return true;
}