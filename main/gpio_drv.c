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
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "iec_controller.h"

#define GPIO_ETH_LED 2
#define GPIO_RS_LED_RX 23
#define GPIO_RS_LED_TX 4
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_ETH_LED) | (1ULL << GPIO_RS_LED_RX) | (1ULL << GPIO_RS_LED_TX))
#define CONFIG_BUTTON 35
#define GPIO_INPUT_PIN_SEL (1ULL << CONFIG_BUTTON)
#define DEBOUNCE 100

static bool iec_slave_connected = false;
static bool rx_led_on = false;

static void led_link(bool state)
{
    gpio_set_level(GPIO_ETH_LED, !state);
}

void set_slave_state_led(bool state)
{
    iec_slave_connected = state;
}

void led_data_rx(void)
{
    gpio_set_level(GPIO_RS_LED_TX, true);
    gpio_set_level(GPIO_RS_LED_RX, false);
    rx_led_on = true;
}

void led_data_tx(void)
{
    gpio_set_level(GPIO_RS_LED_TX, false);
    gpio_set_level(GPIO_RS_LED_RX, true);
}

void led_data_off(void)
{
    gpio_set_level(GPIO_RS_LED_TX, true);
    gpio_set_level(GPIO_RS_LED_RX, true);
}

static void gpio_task(void *arg)
{
    uint32_t count = 0;
    bool delay = false;
    for (;;)
    {
        if (iec_slave_connected)
        {
            led_link(true);
        }
        else
        {
            led_link(count % 4 == false);
        }
        count++;
        if (rx_led_on)
        {
            if (delay)
            {
                delay = rx_led_on = false;
                led_data_off();
            }
            else
                delay = true;
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void gpio_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    led_data_off();
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);
}

bool is_web_cfg(void)
{
    int db_count = DEBOUNCE;
    while (db_count--)
    {
        if (gpio_get_level(CONFIG_BUTTON))
            return false;
    }
    return true;
}