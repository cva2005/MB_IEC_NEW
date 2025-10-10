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
#include "webserver.h"
#include "switch_ota.h"
#include "config.h"

typedef enum
{
    RX_ON_ETH_OFF = 0,
    ETH_ON_RX_OFF = 1,
    TX_ON_ETH_OFF = 2,
    ETH_ON_TX_OFF = 3,
    LED_FULL_CYCLE = 4,
} led_state_t;

#define GPIO_ETH_LED 2
#define GPIO_RS_LED_RX 4
#define GPIO_RS_LED_TX 23
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_ETH_LED) | (1ULL << GPIO_RS_LED_RX) | (1ULL << GPIO_RS_LED_TX))
#define CONFIG_BUTTON 35
#define GPIO_INPUT_PIN_SEL (1ULL << CONFIG_BUTTON)
#define DEBOUNCE 100
#define CONFIG_START 10

static const char *TAG = "GPIO Driver";
static bool iec_slave_connected = false;
static bool rx_led_on = false;
static bool config = false;

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

bool is_button_press(void)
{
    int db_count = DEBOUNCE;
    while (db_count--)
    {
        if (gpio_get_level(CONFIG_BUTTON))
            return false;
    }
    return true;
}

static void gpio_task(void *arg)
{
    uint32_t count = 0;
    bool delay = false;
    led_state_t led_state = RX_ON_ETH_OFF;
    while (true)
    {
        if (is_web_server())
        {
            switch (led_state) {
            case ETH_ON_RX_OFF:
                gpio_set_level(GPIO_ETH_LED, false);
                gpio_set_level(GPIO_RS_LED_RX, true);
                break;
            case TX_ON_ETH_OFF:
                gpio_set_level(GPIO_RS_LED_TX, false);
                gpio_set_level(GPIO_ETH_LED, true);
                break;
            case ETH_ON_TX_OFF:
                gpio_set_level(GPIO_ETH_LED, false);
                gpio_set_level(GPIO_RS_LED_TX, true);
                break;
            default: // RX_ON_ETH_OFF || LED_FULL_CYCLE
                led_state = RX_ON_ETH_OFF;
                gpio_set_level(GPIO_RS_LED_RX, false);
                gpio_set_level(GPIO_ETH_LED, true);
                break;
            }
            led_state++;
        }
        else
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
            if (is_button_press())
            {
                if (!config)
                {
                    ESP_LOGI(TAG, "Start Configuration Reset");
                    count = 0;
                    config = true;
                }
            }
            else
                config = false;
            if (config)
            {
                ESP_LOGI(TAG, "Wait Reset Count: %lu", count);
                if (count == CONFIG_START)
                {
                    ESP_LOGI(TAG, "Start Reset Delay");
                    start_reset_delay(RST_CFG_START);
                    reboot_as_deep_sleep();
                }
            }
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
    xTaskCreate(gpio_task, "gpio_task", 3072, NULL, 10, NULL);
}