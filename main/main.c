#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_http_server.h"
#include "webserver.h"
#include "timer_1ms.h"
#include "config.h"
#include "protocol_examples_common.h"
#include "serial_master.h"
#include "tcp_master.h"
#include "gtw_mb_master.h"
#include "tcp_slave_iec.h"
#include "ser_slave_iec.h"
#include "gtw_params.h"
#include "sdkconfig.h"
#include "gpio_drv.h"
#include "ota_ws_update_esp.h"
#include "esp_ota_ops.h"
#include "switch_ota.h"
#include "arch.h"

const char *TAG = "MB_IEC_GTW";
RTC_DATA_ATTR slave_select_t slave_select = SLAVE_IEC_104_TCP;
RTC_DATA_ATTR static bool ota_key = false;
static bool web_server = false;

bool is_web_server(void)
{
	return web_server;
}

void app_main(void)
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	ESP_ERROR_CHECK(read_config());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	reset_state_t reset_state = get_rst_state();
	clr_rst_state();
	bool cfg_start = false;
	switch (reset_state)
	{
		case FACTORY_LOAD:
			ESP_LOGI(TAG, "Copy Factory app to next part");
			save_arch_event(FW_FCT);
			ota_factory_reload();
			break;
		case ROLLBACK_CMD:
			ESP_LOGI(TAG, "Rollback Firmware to preview part");
			save_arch_event(FW_RLB);
			ota_rollback();
			break;
		case RST_ONLY_CMD:
			ESP_LOGI(TAG, "Reset Command from Browser");
			save_arch_event(RST_WEB);
			break;
		case RST_DEFL_CFG:
			ESP_LOGI(TAG, "Reset to Default Configuration");
			save_arch_event(CFG_DF);
			cfg_start = true;
			break;
		case CLR_ARCH_CMD:
			ESP_LOGI(TAG, "Clear Events Archive");
			save_arch_event(CLR_ARC);
			cfg_start = true;
			break;
		case RST_CFG_START:
			ESP_LOGI(TAG, "Device in to Configuration Mode");
			save_arch_event(CFG_MD);
			cfg_start = true;
			break;
		default:
			if (esp_reset_reason() == ESP_RST_POWERON)
				read_utc_copy();
			save_arch_event(NO_SYS_EVT);
			break;
	}
	gpio_init();
	timer_1ms_init();
	if (!ota_key)
	{
		if (is_button_press() && !cfg_start)
		{
			ota_key = true;
			ESP_LOGI(TAG, "Factory FW Reload over Button Press");
			start_reset_delay(FACTORY_LOAD);
			goto wait_reload;
		}
	}
	ota_key = false;
	web_server = (gtw_param_init() == ESP_ERR_NOT_FOUND) || cfg_start;
	ESP_ERROR_CHECK(example_connect());
	if (check_ota_ws_rollback_enable())
		rollback_ota_ws(false);
	if (web_server)
	{
		webserver_init();
		webserver_start();
		while (true)
		{
		wait_reload:
			if (is_reset_time_out())
			{
				esp_err_t err;
				switch (get_rst_state())
				{
				case FACTORY_LOAD:
					prepare_factory_reload();
					ESP_LOGI(TAG, "Factory Reload Command");
					break;
				case ROLLBACK_CMD:
					if (prepare_rollback() == false)
					{
						ESP_LOGE(TAG, "Rollback Firmware Command: Error");
						goto cmd_false;
					}
					ESP_LOGI(TAG, "Rollback Firmware Command");
					break;
				case RST_ONLY_CMD:
					ESP_LOGI(TAG, "Reset Device Command");
					break;
				case RST_DEFL_CFG:
					err = write_config_default();
					if (err != ESP_OK)
					{
						ESP_LOGE(TAG, "Reset to Default Configuration: Error %d", err);
						goto cmd_false;
					}
					ESP_LOGI(TAG, "Reset to Default Command");
					break;
				case CLR_ARCH_CMD:
					err = clear_events_arch();
					if (err != ESP_OK)
					{
						ESP_LOGE(TAG, "Clear Events Archive: Error %d", err);
						goto cmd_false;
					}
					ESP_LOGI(TAG, "Clear Events Archive");
					break;
				case LD_UTC_CMD:
					save_utc_copy();
					clr_rst_state();
					ESP_LOGI(TAG, "Correct UTC Counter: OK!");
				default:
					goto cmd_false;
				}
				reboot_as_deep_sleep();
			}
		cmd_false:
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}
	mb_serial_master_register();
	mb_tcp_master_register();
	if (slave_select == SLAVE_IEC_101_SERIAL)
	{
		int conn_iec;
		if (!is_mb_connect_use(SER1_CONN))
			conn_iec = SER1_CONN;
		else
		{
			slave_select = SLAVE_IEC_104_TCP;
			goto tcp_only;
		}
		ser_iec_slave_init(conn_iec);
		xTaskCreatePinnedToCore(ser_iec_slave_task,
								"ser_iec_slave_task",
								IEC_TASK_STACK_SIZE,
								NULL,
								1,
								NULL,
								tskNO_AFFINITY);
	}
	else /* slave_select == SLAVE_IEC_104_TCP */
	{
	tcp_only:
		tcp_iec_slave_init();
		xTaskCreatePinnedToCore(tcp_iec_slave_task,
								"tcp_iec_slave_task",
								IEC_TASK_STACK_SIZE,
								NULL,
								1,
								NULL,
								tskNO_AFFINITY);
	}
	mb_master_operation_func();
}